#include "main.h"
#include "GripPush.h"
#include "cmsis_os.h"

/* 换向时先进入停止态，避免继电器直接反向切换 */
#define PUSH_DIRECTION_CHANGE_DELAY_MS 200U

/* 当前推杆状态 */
static PushState_e g_push_state = PUSH_STATE_STOP;

/**
 * @brief 设置推杆继电器为停止态（PA2=0, PA3=0）
 *
 * @return 无
 */
static void push_set_stop(void)
{
    HAL_GPIO_WritePin(PUSH_K1_GPIO_Port, PUSH_K1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PUSH_K2_GPIO_Port, PUSH_K2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 设置推杆继电器为伸出态（PA2=0, PA3=1）
 *
 * @return 无
 */
static void push_set_extend(void)
{
    HAL_GPIO_WritePin(PUSH_K1_GPIO_Port, PUSH_K1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PUSH_K2_GPIO_Port, PUSH_K2_Pin, GPIO_PIN_SET);
}

/**
 * @brief 设置推杆继电器为缩回态（PA2=1, PA3=0）
 *
 * @return 无
 */
static void push_set_retract(void)
{
    HAL_GPIO_WritePin(PUSH_K1_GPIO_Port, PUSH_K1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PUSH_K2_GPIO_Port, PUSH_K2_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 控制推杆伸出（平台前进）
 *
 * 若当前已经是伸出状态则不重复操作。
 * 若当前是缩回状态，则先进入停止态（PA2=0, PA3=0），延时后再切换。
 *
 * @return 无
 */
void platform_forward(void)
{
    if (g_push_state == PUSH_STATE_EXTEND)
    {
        return;
    }

    if (g_push_state == PUSH_STATE_RETRACT)
    {
        push_set_stop();
        osDelay(PUSH_DIRECTION_CHANGE_DELAY_MS);
    }

    push_set_extend();
    g_push_state = PUSH_STATE_EXTEND;
}

/**
 * @brief 控制推杆缩回（平台后退）
 *
 * 若当前已经是缩回状态则不重复操作。
 * 若当前是伸出状态，则先进入停止态（PA2=0, PA3=0），延时后再切换。
 *
 * @return 无
 */
void platform_backward(void)
{
    if (g_push_state == PUSH_STATE_RETRACT)
    {
        return;
    }

    if (g_push_state == PUSH_STATE_EXTEND)
    {
        push_set_stop();
        osDelay(PUSH_DIRECTION_CHANGE_DELAY_MS);
    }

    push_set_retract();
    g_push_state = PUSH_STATE_RETRACT;
}

/**
 * @brief 控制推杆停止
 *
 * @return 无
 */
void platform_stop(void)
{
    push_set_stop();
    g_push_state = PUSH_STATE_STOP;
}

/**
 * @brief 控制夹爪打开
 *
 * @return 无
 */
void claw_open(void)
{
    HAL_GPIO_WritePin(CLAW_GPIO_Port, CLAW_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 控制夹爪关闭
 *
 * @return 无
 */
void claw_close(void)
{
    HAL_GPIO_WritePin(CLAW_GPIO_Port, CLAW_Pin, GPIO_PIN_SET);
}
