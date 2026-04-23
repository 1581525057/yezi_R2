#ifndef __EMM_V5_H
#define __EMM_V5_H
#include <stdbool.h>
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：https://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

#define ABS(x) ((x) > 0 ? (x) : -(x))  // 绝对值宏定义

/* 系统参数枚举 - 步进电机V5版本读取参数 */
typedef enum {
	S_VBUS  = 5,	// 读取总线电压
	S_CBUS  = 6,	// 读取总线电流
	S_CPHA  = 7,	// 读取相电流
	S_ENCO  = 8,	// 读取编码器原始值
	S_CLKC  = 9,	// 读取实时脉冲数
	S_ENCL  = 10,	// 读取经过线性化校准后的编码器值
	S_CLKI  = 11,	// 读取输入脉冲数
	S_TPOS  = 12,	// 读取电机目标位置
	S_SPOS  = 13,	// 读取电机实时设定的目标位置
	S_VEL   = 14,	// 读取电机实时转速
	S_CPOS  = 15,	// 读取电机实时位置
	S_PERR  = 16,	// 读取电机位置误差
	S_VBAT  = 17,	// 读取多圈编码器电池电压（Y42）
	S_TEMP  = 18,	// 读取电机实时温度（Y42）
	S_FLAG  = 19,	// 读取电机状态标志位
	S_OFLAG = 20,	// 读取回零状态标志位
	S_OAF   = 21,	// 读取电机状态标志位 + 回零状态标志位（Y42）
	S_PIN   = 22,	// 读取引脚状态（Y42）
}SysParams_t;

/* 步进电机V5版本配置参数结构体 */
typedef struct {
    uint8_t motor_type;               // 电机类型
    uint8_t pulse_port_mode;          // 脉冲端口模式
    uint8_t comm_port_mode;           // 通信端口模式
    uint8_t en_pin_mode;              // 使能引脚模式
    uint8_t dir_pin_mode;             // 方向引脚模式
    uint8_t microstep;                // 细分数设置
    uint8_t microstep_interp;         // 细分插值模式
    uint8_t reserved;                  // 保留参数
    uint16_t open_loop_current_ma;    // 开环电流，单位：mA
    uint16_t closed_loop_max_current_ma; // 闭环最大电流，单位：mA
    uint16_t max_output_voltage;      // 最大输出电压
    uint8_t uart_baud;                // UART波特率
    uint8_t can_baud;                 // CAN波特率
    uint8_t motor_id;                 // 电机ID
    uint8_t checksum_mode;            // 校验模式
    uint8_t control_ack_mode;         // 控制应答模式
    uint8_t clog_protection_mode;     // 堵转保护模式
    uint16_t clog_detect_speed_rpm;  // 堵转检测速度，单位：RPM
    uint16_t clog_detect_current_ma;  // 堵转检测电流，单位：mA
    uint16_t clog_detect_time_ms;     // 堵转检测时间，单位：ms
    uint16_t pos_window;              // 位置窗口
} EmmV5MotorConfParams_t;

#define		MMCL_LEN		512                 // MMCL指令缓冲区最大长度
#define   STEP_MOTOR_TX_QUEUE_DEPTH      8U    // 步进电机发送队列深度
#define   STEP_MOTOR_TX_FRAME_MAX_LEN    (MMCL_LEN + 5U)  // 步进电机发送帧最大长度
#define   STEP_MOTOR_MIN_GAP_MS          1U    // 步进电机最小发送间隔，单位：ms
#define   STEP_MOTOR_TX_WAIT_RETRY_MS    1U    // 步进电机发送等待重试时间，单位：ms
extern __IO uint16_t MMCL_count, MMCL_cmd[MMCL_LEN];  // MMCL计数器和指令缓冲区

bool StepMotorTx_Submit(const uint8_t *data, uint16_t len);
void StepMotorTx_OnTxComplete(UART_HandleTypeDef *huart);
void StepMotorTx_OnError(UART_HandleTypeDef *huart);
bool StepMotorTx_IsIdle(void);

/**
***********************************************************
***********************************************************
*** 
***
*** @brief	后缀带有（Y42）为Y42新增命令，X42不能用，其他通用
***
*** 
***********************************************************
***********************************************************
***/
/**********************************************************
*** 触发动作命令
**********************************************************/
// 触发编码器校准
void Emm_V5_Trig_Encoder_Cal(uint8_t addr);
// 重启电机（Y42）
void Emm_V5_Reset_Motor(uint8_t addr);
// 将当前位置清零
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);
// 解除堵转保护
void Emm_V5_Reset_Clog_Pro(uint8_t addr);
// 恢复出厂设置
void Emm_V5_Restore_Motor(uint8_t addr);
/**********************************************************
*** 运动控制命令
**********************************************************/
// 多电机命令（Y42）
void Emm_V5_Multi_Motor_Cmd(uint8_t addr);
// 电机使能控制
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);
// 速度模式控制
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
// 位置模式控制
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t motion_mode, bool snF);
// 让电机立即停止运动
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
// 触发多机同步开始运动
void Emm_V5_Synchronous_motion(uint8_t addr);
/**********************************************************
*** 原点回零命令
**********************************************************/
// 设置单圈回零的零点位置
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);
// 触发回零
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
// 强制中断并退出回零
void Emm_V5_Origin_Interrupt(uint8_t addr);
// 读取回零参数
void Emm_V5_Origin_Read_Params(uint8_t addr);
// 修改回零参数
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
/**********************************************************
*** 读取系统参数命令
**********************************************************/
// 定时返回信息命令（Y42）
void Emm_V5_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
// 读取系统参数
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
/**********************************************************
*** 读写驱动参数命令
**********************************************************/
// 修改电机ID地址
void Emm_V5_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id);
// 修改细分值
void Emm_V5_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep);
// 修改掉电标志
void Emm_V5_Modify_PDFlag(uint8_t addr, bool pdf);
// 读取选项参数状态（Y42）
void Emm_V5_Read_Opt_Param_Sta(uint8_t addr);
// 修改电机类型（Y42）
void Emm_V5_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype);
// 修改固件类型（Y42）
void Emm_V5_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype);
// 修改开环/闭环控制模式（Y42）
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode);
// 修改电机运动正方向（Y42）
void Emm_V5_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir);
// 修改锁定按键功能（Y42）
void Emm_V5_Modify_Lock_Btn(uint8_t addr, bool svF, bool lockbtn);
// 修改命令速度值是否缩小10倍输入（Y42）
void Emm_V5_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel);
// 修改开环模式工作电流
void Emm_V5_Modify_OM_ma(uint8_t addr, bool svF, uint16_t om_ma);
// 修改闭环模式最大电流
void Emm_V5_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA);
// 读取PID参数
void Emm_V5_Read_PID_Params(uint8_t addr);
// 修改PID参数
void Emm_V5_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd);
// 读取DMX512协议参数（Y42）
void Emm_V5_Read_DMX512_Params(uint8_t addr);
// 修改DMX512协议参数（Y42）
void Emm_V5_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step);
// 读取位置到达窗口（Y42）
void Emm_V5_Read_Pos_Window(uint8_t addr);
// 修改位置到达窗口（Y42）
void Emm_V5_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw);
// 读取过热过流保护检测阈值（Y42）
void Emm_V5_Read_Otocp(uint8_t addr);
// 修改过热过流保护检测阈值（Y42）
void Emm_V5_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms);
// 读取心跳保护功能时间（Y42）
void Emm_V5_Read_Heart_Protect(uint8_t addr);
// 修改心跳保护功能时间（Y42）
void Emm_V5_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp);
// 读取积分限幅/刚性系数（Y42）
void Emm_V5_Read_Integral_Limit(uint8_t addr);
// 修改积分限幅/刚性系数（Y42）
void Emm_V5_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il);
// 修改完整驱动配置参数（Y42）
void Emm_V5_Modify_Motor_Conf_Params(uint8_t addr, bool svF, const EmmV5MotorConfParams_t *params);
/**********************************************************
*** 读取所有驱动参数命令
**********************************************************/
// 读取系统状态参数
void Emm_V5_Read_System_State_Params(uint8_t addr);
// 读取驱动配置参数
void Emm_V5_Read_Motor_Conf_Params(uint8_t addr);

/**
***********************************************************
***********************************************************
*** 
***
*** @brief	以下是把相应命令加载到Y42多电机命令上的函数（Y42）
***
*** 
***********************************************************
***********************************************************
***/
/**********************************************************
*** 触发动作命令
**********************************************************/
// 触发编码器校准 - 加载到多电机指令上
void Emm_V5_MMCL_Trig_Encoder_Cal(uint8_t addr);
// 重启电机 - 加载到多电机指令上
void Emm_V5_MMCL_Reset_Motor(uint8_t addr);
// 将当前位置清零 - 加载到多电机指令上
void Emm_V5_MMCL_Reset_CurPos_To_Zero(uint8_t addr);
// 解除堵转保护 - 加载到多电机指令上
void Emm_V5_MMCL_Reset_Clog_Pro(uint8_t addr);
// 恢复出厂设置 - 加载到多电机指令上
void Emm_V5_MMCL_Restore_Motor(uint8_t addr);
/**********************************************************
*** 运动控制命令
**********************************************************/
// 电机使能控制 - 加载到多电机指令上
void Emm_V5_MMCL_En_Control(uint8_t addr, bool state, bool snF);
// 速度模式控制 - 加载到多电机指令上
void Emm_V5_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
// 位置模式控制 - 加载到多电机指令上
void Emm_V5_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, uint8_t motion_mode, bool snF);
// 让电机立即停止运动 - 加载到多电机指令上
void Emm_V5_MMCL_Stop_Now(uint8_t addr, bool snF);
// 触发多机同步开始运动 - 加载到多电机指令上
void Emm_V5_MMCL_Synchronous_motion(uint8_t addr);
/**********************************************************
*** 原点回零命令
**********************************************************/
// 设置单圈回零的零点位置 - 加载到多电机指令上
void Emm_V5_MMCL_Origin_Set_O(uint8_t addr, bool svF);
// 触发回零 - 加载到多电机指令上
void Emm_V5_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
// 强制中断并退出回零 - 加载到多电机指令上
void Emm_V5_MMCL_Origin_Interrupt(uint8_t addr);
// 修改回零参数 - 加载到多电机指令上
void Emm_V5_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
/**********************************************************
*** 读取系统参数命令
**********************************************************/
// 定时返回信息命令（Y42） - 加载到多电机指令上
void Emm_V5_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
// 读取系统参数 - 加载到多电机指令上
void Emm_V5_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s);
/**********************************************************
*** 读写驱动参数命令
**********************************************************/

#ifdef __cplusplus
}
#endif

#endif
