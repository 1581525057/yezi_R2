#ifndef YUN_J60_H
#define YUN_J60_H

#include "stm32h7xx.h"
#include "bsp_can.h"

/**
 * @brief J60 电机反馈数据结构体
 */
typedef struct
{
	float position;	   // 位置
	float velocity;	   // 速度
	float torque;	   // 力矩
	float temperature; // 温度
} Dog_j60_motor_t;

/**
 * @brief
 *        云深处 J60 电机类
 *        负责：
 *        1、J60 电机反馈数据保存
 *        2、使能 / 控制命令发送
 *        3、CAN 数据反馈解析
 */
class Yun_J60_Class
{
public:
	/* 电机对象 --------------------------------------------------------------*/
	static Dog_j60_motor_t motor[8];

	/* 电机使能状态，协议允许 id 0~31 ---------------------------------------*/
	static uint8_t motor_enabled[32];

public:
	/* 对外接口 --------------------------------------------------------------*/

	/**
	 * @brief  使能指定 J60 电机
	 * @param  id : 电机 id
	 * @retval None
	 */
	static void EnableMotor(uint8_t id);

	/**
	 * @brief  发送 J60 控制命令
	 * @param  id    : 电机 id
	 * @param  p_des : 目标位置
	 * @param  v_des : 目标速度
	 * @param  kp    : 位置环刚度
	 * @param  kd    : 阻尼
	 * @param  t_ff  : 前馈力矩
	 * @retval None
	 */
	static void SendControl(uint8_t id,
							float p_des,
							float v_des,
							float kp,
							float kd,
							float t_ff);

	/**
	 * @brief  J60 接收处理函数
	 * @param  fdcan_rxframe : 接收帧结构体
	 * @param  Data          : 接收数据
	 * @retval None
	 */
	static void RxHandler(FDCAN_RxFrame_TypeDef *fdcan_rxframe, uint8_t Data[8]);

private:
	/* 内部工具函数 ----------------------------------------------------------*/

	/**
	 * @brief  浮点数转无符号整型，用在发送控制数据打包
	 * @param  x     : 输入浮点数
	 * @param  x_min : 最小值
	 * @param  x_max : 最大值
	 * @param  bits  : 位宽
	 * @retval uint16_t
	 */
	static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

	/**
	 * @brief  无符号整型转回浮点数，用在接收反馈数据解包
	 * @param  x     : 输入整数
	 * @param  x_min : 最小值
	 * @param  x_max : 最大值
	 * @param  bits  : 位宽
	 * @retval float
	 */
	static float uint_to_float(uint32_t x, float x_min, float x_max, int bits);

	/**
	 * @brief  解析 J60 反馈数据
	 * @param  id     : 电机 id
	 * @param  rxData : 接收数据
	 * @retval None
	 */
	static void DecodeFeedback(uint8_t id, uint8_t *rxData);

	/**
	 * @brief  把 FDCAN DLC 宏转换成字节数
	 * @param  DataLength : FDCAN Header.DataLength
	 * @retval 字节数
	 */
	static uint8_t GetDataLengthBytes(uint32_t DataLength);
};

extern Yun_J60_Class yun_j60_motor;
#endif