/**
  ******************************************************************************
  * @file    EmmV5.cpp
  * @brief   Emm_V5步进电机驱动实现文件
  * @details 实现步进电机UART发送队列、DMA传输控制及命令封装函数
  * @note    使用FreeRTOS定时器和临界区保护，支持多电机同步控制
  ******************************************************************************
  */

#include "EmmV5.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* 步进电机 UART 发送接口 */
#define MotorHuart huart9

typedef struct
{
  uint16_t len;
  uint8_t data[STEP_MOTOR_TX_FRAME_MAX_LEN];
} StepMotorTxFrame_t;

static StepMotorTxFrame_t g_step_motor_tx_queue[STEP_MOTOR_TX_QUEUE_DEPTH];
static volatile uint16_t g_step_motor_tx_head = 0U;
static volatile uint16_t g_step_motor_tx_tail = 0U;
static volatile uint16_t g_step_motor_tx_count = 0U;
static volatile uint8_t g_step_motor_tx_dma_active = 0U;
static volatile uint32_t g_step_motor_tx_error_count = 0U;
static volatile uint32_t g_step_motor_tx_overflow_count = 0U;
static volatile uint32_t g_step_motor_last_tx_done_tick = 0U;
#if (STEP_MOTOR_MIN_GAP_MS > 0U)
static StaticTimer_t g_step_motor_gap_timer_buffer;
static TimerHandle_t g_step_motor_gap_timer = NULL;
static volatile uint8_t g_step_motor_gap_timer_armed = 0U;
#endif

__IO uint16_t MMCL_count = 0, MMCL_cmd[MMCL_LEN] = {0};

/* 步进电机发送队列辅助函数 */
static uint32_t StepMotorTx_EnterCritical(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void StepMotorTx_ExitCritical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static bool StepMotorTx_CanWait(void)
{
  return (bool)((__get_IPSR() == 0U) &&
                (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED));
}

static void StepMotorTx_Kick(void);

#if (STEP_MOTOR_MIN_GAP_MS > 0U)
static void StepMotorTx_GapTimerCallback(TimerHandle_t xTimer)
{
  uint32_t primask = 0U;

  (void)xTimer;

  primask = StepMotorTx_EnterCritical();
  g_step_motor_gap_timer_armed = 0U;
  StepMotorTx_ExitCritical(primask);

  StepMotorTx_Kick();
}

static void StepMotorTx_EnsureGapTimerCreated(void)
{
  if (g_step_motor_gap_timer != NULL)
  {
    return;
  }

  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
  {
    return;
  }

  taskENTER_CRITICAL();
  if (g_step_motor_gap_timer == NULL)
  {
    g_step_motor_gap_timer = xTimerCreateStatic("stp_gap",
                                                pdMS_TO_TICKS(STEP_MOTOR_MIN_GAP_MS),
                                                pdFALSE,
                                                NULL,
                                                StepMotorTx_GapTimerCallback,
                                                &g_step_motor_gap_timer_buffer);
  }
  taskEXIT_CRITICAL();
}

static void StepMotorTx_ArmGapTimerFromIsr(void)
{
  BaseType_t higher_priority_task_woken = pdFALSE;
  uint32_t primask = 0U;

  if (g_step_motor_gap_timer == NULL)
  {
    return;
  }

  primask = StepMotorTx_EnterCritical();
  if (g_step_motor_gap_timer_armed != 0U)
  {
    StepMotorTx_ExitCritical(primask);
    return;
  }
  g_step_motor_gap_timer_armed = 1U;
  StepMotorTx_ExitCritical(primask);

  if (xTimerChangePeriodFromISR(g_step_motor_gap_timer,
                                pdMS_TO_TICKS(STEP_MOTOR_MIN_GAP_MS),
                                &higher_priority_task_woken) != pdPASS)
  {
    primask = StepMotorTx_EnterCritical();
    g_step_motor_gap_timer_armed = 0U;
    ++g_step_motor_tx_error_count;
    StepMotorTx_ExitCritical(primask);
    return;
  }

  portYIELD_FROM_ISR(higher_priority_task_woken);
}
#endif

static void StepMotorTx_Kick(void)
{
  uint16_t frame_len = 0U;
  uint8_t *frame_data = NULL;
  HAL_StatusTypeDef status = HAL_ERROR;
  uint32_t primask = 0U;
#if (STEP_MOTOR_MIN_GAP_MS > 0U)
  const uint32_t now_tick = HAL_GetTick();
#endif

#if (STEP_MOTOR_MIN_GAP_MS > 0U)
  if ((g_step_motor_tx_count > 0U) &&
      ((now_tick - g_step_motor_last_tx_done_tick) < STEP_MOTOR_MIN_GAP_MS))
  {
    return;
  }
#endif

  primask = StepMotorTx_EnterCritical();
  if ((g_step_motor_tx_count == 0U) || (g_step_motor_tx_dma_active != 0U))
  {
    StepMotorTx_ExitCritical(primask);
    return;
  }

  g_step_motor_tx_dma_active = 1U;
#if (STEP_MOTOR_MIN_GAP_MS > 0U)
  g_step_motor_gap_timer_armed = 0U;
#endif
  frame_len = g_step_motor_tx_queue[g_step_motor_tx_head].len;
  frame_data = g_step_motor_tx_queue[g_step_motor_tx_head].data;
  StepMotorTx_ExitCritical(primask);

  status = HAL_UART_Transmit_DMA(&MotorHuart, frame_data, frame_len);
  if (status == HAL_OK)
  {
    return;
  }

  primask = StepMotorTx_EnterCritical();
  g_step_motor_tx_dma_active = 0U;
  ++g_step_motor_tx_error_count;
  StepMotorTx_ExitCritical(primask);
}

bool StepMotorTx_Submit(const uint8_t *data, uint16_t len)
{
  uint32_t primask = 0U;
  TickType_t wait_ticks = pdMS_TO_TICKS(STEP_MOTOR_TX_WAIT_RETRY_MS);

#if (STEP_MOTOR_MIN_GAP_MS > 0U)
  StepMotorTx_EnsureGapTimerCreated();
#endif

  if (wait_ticks == 0U)
  {
    wait_ticks = 1U;
  }

  if ((data == NULL) || (len == 0U) || (len > STEP_MOTOR_TX_FRAME_MAX_LEN))
  {
    primask = StepMotorTx_EnterCritical();
    ++g_step_motor_tx_error_count;
    StepMotorTx_ExitCritical(primask);
    return false;
  }

  for (;;)
  {
    primask = StepMotorTx_EnterCritical();
    if (g_step_motor_tx_count < STEP_MOTOR_TX_QUEUE_DEPTH)
    {
      memcpy(g_step_motor_tx_queue[g_step_motor_tx_tail].data, data, len);
      g_step_motor_tx_queue[g_step_motor_tx_tail].len = len;
      g_step_motor_tx_tail = (uint16_t)((g_step_motor_tx_tail + 1U) % STEP_MOTOR_TX_QUEUE_DEPTH);
      ++g_step_motor_tx_count;
      StepMotorTx_ExitCritical(primask);

      StepMotorTx_Kick();
      return true;
    }

    StepMotorTx_ExitCritical(primask);

    if (StepMotorTx_CanWait() == false)
    {
      primask = StepMotorTx_EnterCritical();
      ++g_step_motor_tx_overflow_count;
      StepMotorTx_ExitCritical(primask);
      return false;
    }

    StepMotorTx_Kick();
    vTaskDelay(wait_ticks);
  }
}

void StepMotorTx_OnTxComplete(UART_HandleTypeDef *huart)
{
  uint32_t primask = 0U;
  uint8_t has_pending_frame = 0U;

  if (huart != &MotorHuart)
  {
    return;
  }

  primask = StepMotorTx_EnterCritical();
  if (g_step_motor_tx_count > 0U)
  {
    g_step_motor_tx_head = (uint16_t)((g_step_motor_tx_head + 1U) % STEP_MOTOR_TX_QUEUE_DEPTH);
    --g_step_motor_tx_count;
  }

  g_step_motor_tx_dma_active = 0U;
  g_step_motor_last_tx_done_tick = HAL_GetTick();
  has_pending_frame = (uint8_t)(g_step_motor_tx_count > 0U);
  StepMotorTx_ExitCritical(primask);

#if (STEP_MOTOR_MIN_GAP_MS > 0U)
  if (has_pending_frame != 0U)
  {
    StepMotorTx_ArmGapTimerFromIsr();
  }
#else
  (void)has_pending_frame;
  StepMotorTx_Kick();
#endif
}

void StepMotorTx_OnError(UART_HandleTypeDef *huart)
{
  uint32_t primask = 0U;

  if ((huart != &MotorHuart) ||
      (g_step_motor_tx_dma_active == 0U) ||
      (huart->hdmatx == NULL) ||
      (huart->hdmatx->ErrorCode == HAL_DMA_ERROR_NONE))
  {
    return;
  }

  primask = StepMotorTx_EnterCritical();
  g_step_motor_tx_dma_active = 0U;
  ++g_step_motor_tx_error_count;
  StepMotorTx_ExitCritical(primask);

  StepMotorTx_Kick();
}

bool StepMotorTx_IsIdle(void)
{
  uint32_t primask = 0U;
  bool is_idle = false;

  primask = StepMotorTx_EnterCritical();
  is_idle = (bool)((g_step_motor_tx_count == 0U) && (g_step_motor_tx_dma_active == 0U)
#if (STEP_MOTOR_MIN_GAP_MS > 0U)
                   && (g_step_motor_gap_timer_armed == 0U)
#endif
                   );
  StepMotorTx_ExitCritical(primask);

  return is_idle;
}

/* 串口 DMA 发送完成与异常回调 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  StepMotorTx_OnTxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  StepMotorTx_OnError(huart);
}

#define HAL_UART_Transmit_DMA(huart, pData, Size) StepMotorTx_Submit((const uint8_t *)(pData), (uint16_t)(Size))

/* 触发动作命令 */
void Emm_V5_Trig_Encoder_Cal(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x06;
  cmd[2] =  0x45;
  cmd[3] =  0x6B;

	HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

void Emm_V5_Reset_Motor(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x08;
  cmd[2] =  0x97;
  cmd[3] =  0x6B;

	HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0A;
  cmd[2] =  0x6D;
  cmd[3] =  0x6B;

	HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0E;
  cmd[2] =  0x52;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

void Emm_V5_Restore_Motor(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0F;
  cmd[2] =  0x5F;
  cmd[3] =  0x6B;

	HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

/* 运动控制命令 */
void Emm_V5_Multi_Motor_Cmd(uint8_t addr)
{
  uint16_t i = 0, j = 0, len = 0; __IO static uint8_t cmd[STEP_MOTOR_TX_FRAME_MAX_LEN] = {0};

	if(MMCL_count > 0)
	{

		len = MMCL_count + 5;

		cmd[0] = addr;
		cmd[1] = 0xAA;
		cmd[2] = (uint8_t)(len >> 8);
		cmd[3] = (uint8_t)(len);
		for(i=0,j=4; i < MMCL_count; i++,j++) { cmd[j] = MMCL_cmd[i]; }
		cmd[j] = 0x6B; ++j;

		if (StepMotorTx_Submit((const uint8_t *)cmd, j))
		{
			MMCL_count = 0;
		}
	}
	else
	{
		MMCL_count = 0;
	}
}

void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF3;
  cmd[2] =  0xAB;
  cmd[3] =  (uint8_t)state;
  cmd[4] =  snF;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 6);
}

void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF6;
  cmd[2] =  dir;
  cmd[3] =  (uint8_t)(vel >> 8);
  cmd[4] =  (uint8_t)(vel >> 0);
  cmd[5] =  acc;
  cmd[6] =  snF;
  cmd[7] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 8);
}

void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t motion_mode, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0xFD;
  cmd[2]  =  dir;
  cmd[3]  =  (uint8_t)(vel >> 8);
  cmd[4]  =  (uint8_t)(vel >> 0);
  cmd[5]  =  acc;
  cmd[6]  =  (uint8_t)(clk >> 24);
  cmd[7]  =  (uint8_t)(clk >> 16);
  cmd[8]  =  (uint8_t)(clk >> 8);
  cmd[9]  =  (uint8_t)(clk >> 0);
  cmd[10] =  motion_mode;
  cmd[11] =  snF;
  cmd[12] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 13);
}

void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFE;
  cmd[2] =  0x98;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 5);
}

void Emm_V5_Synchronous_motion(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFF;
  cmd[2] =  0x66;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

/* 原点回零命令 */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x93;
  cmd[2] =  0x88;
  cmd[3] =  svF;
  cmd[4] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 5);
}

void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x9A;
  cmd[2] =  o_mode;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 5);
}

void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x9C;
  cmd[2] =  0x48;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

void Emm_V5_Origin_Read_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x22;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 3);
}

void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  __IO static uint8_t cmd[32] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x4C;
  cmd[2] =  0xAE;
  cmd[3] =  svF;
  cmd[4] =  o_mode;
  cmd[5] =  o_dir;
  cmd[6]  =  (uint8_t)(o_vel >> 8);
  cmd[7]  =  (uint8_t)(o_vel >> 0);
  cmd[8]  =  (uint8_t)(o_tm >> 24);
  cmd[9]  =  (uint8_t)(o_tm >> 16);
  cmd[10] =  (uint8_t)(o_tm >> 8);
  cmd[11] =  (uint8_t)(o_tm >> 0);
  cmd[12] =  (uint8_t)(sl_vel >> 8);
  cmd[13] =  (uint8_t)(sl_vel >> 0);
  cmd[14] =  (uint8_t)(sl_ma >> 8);
  cmd[15] =  (uint8_t)(sl_ma >> 0);
  cmd[16] =  (uint8_t)(sl_ms >> 8);
  cmd[17] =  (uint8_t)(sl_ms >> 0);
  cmd[18] =  potF;
  cmd[19] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 20);
}

/* 读取系统参数命令 */
void Emm_V5_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
{
  uint8_t i = 0; __IO static uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;

  cmd[i] = 0x11; ++i;

  cmd[i] = 0x18; ++i;

  switch(s)
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;
		case S_CBUS : cmd[i] = 0x26; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
		case S_ENCO : cmd[i] = 0x29; ++i; break;
		case S_CLKC : cmd[i] = 0x30; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
		case S_CLKI : cmd[i] = 0x32; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_SPOS : cmd[i] = 0x34; ++i; break;
		case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
		case S_VBAT : cmd[i] = 0x38; ++i; break;
		case S_TEMP : cmd[i] = 0x39; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;
		case S_OAF  : cmd[i] = 0x3C; ++i; break;
		case S_PIN  : cmd[i] = 0x3D; ++i; break;
    default: break;
  }

	cmd[i] = (uint8_t)(time_ms >> 8);  ++i;
	cmd[i] = (uint8_t)(time_ms >> 0);  ++i;

  cmd[i] = 0x6B; ++i;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, i);
}

void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0; __IO static uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;

  switch(s)
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;
		case S_CBUS : cmd[i] = 0x26; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
		case S_ENCO : cmd[i] = 0x29; ++i; break;
		case S_CLKC : cmd[i] = 0x30; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
		case S_CLKI : cmd[i] = 0x32; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_SPOS : cmd[i] = 0x34; ++i; break;
		case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
		case S_VBAT : cmd[i] = 0x38; ++i; break;
		case S_TEMP : cmd[i] = 0x39; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;
		case S_OAF  : cmd[i] = 0x3C; ++i; break;
		case S_PIN  : cmd[i] = 0x3D; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, i);
}

/* 读写驱动参数命令 */
void Emm_V5_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xAE;
  cmd[2] =  0x4B;
  cmd[3] =  svF;
  cmd[4] =  id;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 6);
}

void Emm_V5_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x84;
  cmd[2] =  0x8A;
  cmd[3] =  svF;
  cmd[4] =  mstep;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 6);
}

void Emm_V5_Modify_PDFlag(uint8_t addr, bool pdf)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x50;
  cmd[2] =  pdf;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

void Emm_V5_Read_Opt_Param_Sta(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x1A;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 3);
}

void Emm_V5_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype)
{
  __IO static uint8_t cmd[16] = {0}; uint8_t MotType = 0;

	if(mottype) { MotType = 25; } else { MotType = 50; }

  cmd[0] =  addr;
  cmd[1] =  0xD7;
  cmd[2] =  0x35;
  cmd[3] =  svF;
  cmd[4] =  MotType;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 6);
}

void Emm_V5_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xD5;
  cmd[2] =  0x69;
  cmd[3] =  svF;
  cmd[4] =  fwtype;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 6);
}

void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x46;
  cmd[2] =  0x69;
  cmd[3] =  svF;
  cmd[4] =  ctrl_mode;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 6);
}

void Emm_V5_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xD4;
  cmd[2] =  0x60;
  cmd[3] =  svF;
  cmd[4] =  dir;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 6);
}

void Emm_V5_Modify_Lock_Btn(uint8_t addr, bool svF, bool lock)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xD0;
  cmd[2] =  0xB3;
  cmd[3] =  svF;
  cmd[4] =  lock;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 6);
}

void Emm_V5_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x4F;
  cmd[2] =  0x71;
  cmd[3] =  svF;
  cmd[4] =  s_vel;
  cmd[5] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 6);
}

void Emm_V5_Modify_OM_mA(uint8_t addr, bool svF, uint16_t om_ma)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x44;
  cmd[2] =  0x33;
  cmd[3] =  svF;
  cmd[4] =  (uint8_t)(om_ma >> 8);
	cmd[5] =  (uint8_t)(om_ma >> 0);
  cmd[6] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 7);
}

void Emm_V5_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x45;
  cmd[2] =  0x66;
  cmd[3] =  svF;
  cmd[4] =  (uint8_t)(foc_mA >> 8);
	cmd[5] =  (uint8_t)(foc_mA >> 0);
  cmd[6] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 7);
}

void Emm_V5_Read_PID_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x21;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 3);
}

void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd)
{
  __IO static uint8_t cmd[20] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0x4A;
  cmd[2]  =  0xC3;
  cmd[3]  =  svF;
  cmd[4]  =  (uint8_t)(kp >> 24);
	cmd[5]  =  (uint8_t)(kp >> 16);
	cmd[6]  =  (uint8_t)(kp >> 8);
	cmd[7]  =  (uint8_t)(kp >> 0);
	cmd[8]  =  (uint8_t)(ki >> 24);
	cmd[9]  =  (uint8_t)(ki >> 16);
	cmd[10] =  (uint8_t)(ki >> 8);
	cmd[11] =  (uint8_t)(ki >> 0);
	cmd[12] =  (uint8_t)(kd >> 24);
	cmd[13] =  (uint8_t)(kd >> 16);
	cmd[14] =  (uint8_t)(kd >> 8);
	cmd[15] =  (uint8_t)(kd >> 0);
  cmd[16] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 17);
}

void Emm_V5_Read_DMX512_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x49;
	cmd[2] =  0x78;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

void Emm_V5_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step)
{
  __IO static uint8_t cmd[32] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0xD9;
  cmd[2]  =  0x90;
  cmd[3]  =  svF;
  cmd[4]  =  (uint8_t)(tch >> 8);
  cmd[5]  =  (uint8_t)(tch >> 0);
	cmd[6]  =  nch;
	cmd[7]  =  mode;
	cmd[8]  =  (uint8_t)(vel >> 8);
  cmd[9]  =  (uint8_t)(vel >> 0);
	cmd[10] =  (uint8_t)(acc >> 8);
  cmd[11] =  (uint8_t)(acc >> 0);
	cmd[12] =  (uint8_t)(vel_step >> 8);
  cmd[13] =  (uint8_t)(vel_step >> 0);
  cmd[14]  = (uint8_t)(pos_step >> 24);
  cmd[15]  = (uint8_t)(pos_step >> 16);
  cmd[16] =  (uint8_t)(pos_step >> 8);
  cmd[17] =  (uint8_t)(pos_step >> 0);
  cmd[18] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 19);
}

void Emm_V5_Read_Pos_Window(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x41;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 3);
}

void Emm_V5_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xD1;
  cmd[2] =  0x07;
  cmd[3] =  svF;
  cmd[4] =  (uint8_t)(prw >> 8);
	cmd[5] =  (uint8_t)(prw >> 0);
  cmd[6] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 7);
}

void Emm_V5_Read_Otocp(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x13;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 3);
}

void Emm_V5_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0xD3;
  cmd[2]  =  0x56;
  cmd[3]  =  svF;
  cmd[4]  =  (uint8_t)(otp >> 8);
	cmd[5]  =  (uint8_t)(otp >> 0);
	cmd[6]  =  (uint8_t)(ocp >> 8);
	cmd[7]  =  (uint8_t)(ocp >> 0);
	cmd[8]  =  (uint8_t)(time_ms >> 8);
	cmd[9]  =  (uint8_t)(time_ms >> 0);
  cmd[10] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 11);
}

void Emm_V5_Read_Heart_Protect(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x16;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 3);
}

void Emm_V5_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0x68;
  cmd[2]  =  0x38;
  cmd[3]  =  svF;
  cmd[4]  =  (uint8_t)(hp >> 24);
	cmd[5]  =  (uint8_t)(hp >> 16);
	cmd[6]  =  (uint8_t)(hp >> 8);
	cmd[7]  =  (uint8_t)(hp >> 0);
  cmd[8]  =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 9);
}

void Emm_V5_Read_Integral_Limit(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x23;
  cmd[2] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 3);
}

void Emm_V5_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0x4B;
  cmd[2]  =  0x57;
  cmd[3]  =  svF;
  cmd[4]  =  (uint8_t)(il >> 24);
	cmd[5]  =  (uint8_t)(il >> 16);
	cmd[6]  =  (uint8_t)(il >> 8);
	cmd[7]  =  (uint8_t)(il >> 0);
  cmd[8]  =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 9);
}

void Emm_V5_Modify_Motor_Conf_Params(uint8_t addr, bool svF, const EmmV5MotorConfParams_t *params)
{
  __IO static uint8_t cmd[40] = {0};

  if (params == NULL)
  {
    return;
  }

  cmd[0]  =  addr;
  cmd[1]  =  0x48;
  cmd[2]  =  0xD1;
  cmd[3]  =  svF;
  cmd[4]  =  params->motor_type;
  cmd[5]  =  params->pulse_port_mode;
  cmd[6]  =  params->comm_port_mode;
  cmd[7]  =  params->en_pin_mode;
  cmd[8]  =  params->dir_pin_mode;
  cmd[9]  =  params->microstep;
  cmd[10] =  params->microstep_interp;
  cmd[11] =  params->reserved;
  cmd[12] =  (uint8_t)(params->open_loop_current_ma >> 8);
  cmd[13] =  (uint8_t)(params->open_loop_current_ma >> 0);
  cmd[14] =  (uint8_t)(params->closed_loop_max_current_ma >> 8);
  cmd[15] =  (uint8_t)(params->closed_loop_max_current_ma >> 0);
  cmd[16] =  (uint8_t)(params->max_output_voltage >> 8);
  cmd[17] =  (uint8_t)(params->max_output_voltage >> 0);
  cmd[18] =  params->uart_baud;
  cmd[19] =  params->can_baud;
  cmd[20] =  params->motor_id;
  cmd[21] =  params->checksum_mode;
  cmd[22] =  params->control_ack_mode;
  cmd[23] =  params->clog_protection_mode;
  cmd[24] =  (uint8_t)(params->clog_detect_speed_rpm >> 8);
  cmd[25] =  (uint8_t)(params->clog_detect_speed_rpm >> 0);
  cmd[26] =  (uint8_t)(params->clog_detect_current_ma >> 8);
  cmd[27] =  (uint8_t)(params->clog_detect_current_ma >> 0);
  cmd[28] =  (uint8_t)(params->clog_detect_time_ms >> 8);
  cmd[29] =  (uint8_t)(params->clog_detect_time_ms >> 0);
  cmd[30] =  (uint8_t)(params->pos_window >> 8);
  cmd[31] =  (uint8_t)(params->pos_window >> 0);
  cmd[32] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 33);
}

/* 读取所有驱动参数命令 */
void Emm_V5_Read_System_State_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x43;
	cmd[2] =  0x7A;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

void Emm_V5_Read_Motor_Conf_Params(uint8_t addr)
{
  __IO static uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x42;
	cmd[2] =  0x6C;
  cmd[3] =  0x6B;

  HAL_UART_Transmit_DMA(&MotorHuart, (uint8_t *)cmd, 4);
}

/* 多电机命令加载接口 */
void Emm_V5_MMCL_Trig_Encoder_Cal(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x06;
  cmd[2] =  0x45;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Reset_Motor(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x08;
  cmd[2] =  0x97;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0A;
  cmd[2] =  0x6D;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0E;
  cmd[2] =  0x52;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Restore_Motor(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0F;
  cmd[2] =  0x5F;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/* 多电机运动控制命令 */
void Emm_V5_MMCL_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF3;
  cmd[2] =  0xAB;
  cmd[3] =  (uint8_t)state;
  cmd[4] =  snF;
  cmd[5] =  0x6B;

  for(j=0; j < 6; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF6;
  cmd[2] =  dir;
  cmd[3] =  (uint8_t)(vel >> 8);
  cmd[4] =  (uint8_t)(vel >> 0);
  cmd[5] =  acc;
  cmd[6] =  snF;
  cmd[7] =  0x6B;

  for(j=0; j < 8; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t motion_mode, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0xFD;
  cmd[2]  =  dir;
  cmd[3]  =  (uint8_t)(vel >> 8);
  cmd[4]  =  (uint8_t)(vel >> 0);
  cmd[5]  =  acc;
  cmd[6]  =  (uint8_t)(clk >> 24);
  cmd[7]  =  (uint8_t)(clk >> 16);
  cmd[8]  =  (uint8_t)(clk >> 8);
  cmd[9]  =  (uint8_t)(clk >> 0);
  cmd[10] =  motion_mode;
  cmd[11] =  snF;
  cmd[12] =  0x6B;

  for(j=0; j < 13; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFE;
  cmd[2] =  0x98;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Synchronous_motion(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFF;
  cmd[2] =  0x66;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/* 多电机原点回零命令 */
void Emm_V5_MMCL_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x93;
  cmd[2] =  0x88;
  cmd[3] =  svF;
  cmd[4] =  0x6B;

  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x9A;
  cmd[2] =  o_mode;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  for(j=0; j < 5; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Origin_Interrupt(uint8_t addr)
{
  uint8_t j = 0, cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x9C;
  cmd[2] =  0x48;
  cmd[3] =  0x6B;

  for(j=0; j < 4; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t j = 0, cmd[32] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x4C;
  cmd[2] =  0xAE;
  cmd[3] =  svF;
  cmd[4] =  o_mode;
  cmd[5] =  o_dir;
  cmd[6]  =  (uint8_t)(o_vel >> 8);
  cmd[7]  =  (uint8_t)(o_vel >> 0);
  cmd[8]  =  (uint8_t)(o_tm >> 24);
  cmd[9]  =  (uint8_t)(o_tm >> 16);
  cmd[10] =  (uint8_t)(o_tm >> 8);
  cmd[11] =  (uint8_t)(o_tm >> 0);
  cmd[12] =  (uint8_t)(sl_vel >> 8);
  cmd[13] =  (uint8_t)(sl_vel >> 0);
  cmd[14] =  (uint8_t)(sl_ma >> 8);
  cmd[15] =  (uint8_t)(sl_ma >> 0);
  cmd[16] =  (uint8_t)(sl_ms >> 8);
  cmd[17] =  (uint8_t)(sl_ms >> 0);
  cmd[18] =  potF;
  cmd[19] =  0x6B;

  for(j=0; j < 20; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

/* 多电机系统参数命令 */
void Emm_V5_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms)
{
  uint8_t i = 0, j = 0; uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;

  cmd[i] = 0x11; ++i;

  cmd[i] = 0x18; ++i;

  switch(s)
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;
		case S_CBUS : cmd[i] = 0x26; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
		case S_ENCO : cmd[i] = 0x29; ++i; break;
		case S_CLKC : cmd[i] = 0x30; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
		case S_CLKI : cmd[i] = 0x32; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_SPOS : cmd[i] = 0x34; ++i; break;
		case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
		case S_VBAT : cmd[i] = 0x38; ++i; break;
		case S_TEMP : cmd[i] = 0x39; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;
		case S_OAF  : cmd[i] = 0x3C; ++i; break;
		case S_PIN  : cmd[i] = 0x3D; ++i; break;
    default: break;
  }

	cmd[i] = (uint8_t)(time_ms >> 8);  ++i;
	cmd[i] = (uint8_t)(time_ms >> 0);  ++i;

  cmd[i] = 0x6B; ++i;

  for(j=0; j < i; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}

void Emm_V5_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0, j = 0; uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;

  switch(s)
  {
    case S_VBUS : cmd[i] = 0x24; ++i; break;
		case S_CBUS : cmd[i] = 0x26; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
		case S_ENCO : cmd[i] = 0x29; ++i; break;
		case S_CLKC : cmd[i] = 0x30; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
		case S_CLKI : cmd[i] = 0x32; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_SPOS : cmd[i] = 0x34; ++i; break;
		case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
		case S_VBAT : cmd[i] = 0x38; ++i; break;
		case S_TEMP : cmd[i] = 0x39; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_OFLAG: cmd[i] = 0x3B; ++i; break;
		case S_OAF  : cmd[i] = 0x3C; ++i; break;
		case S_PIN  : cmd[i] = 0x3D; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;

  for(j=0; j < i; j++) { MMCL_cmd[MMCL_count] = cmd[j]; ++MMCL_count; }
}
