#include "chassis_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "omni_chassis.h"
#include "dji_motor.h"
#include "bsp_remove.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "PID.h"
#include "yun_j60.h"
#include "dm_imu.h"
#include "lift_auto.h"
#include "VescMotor.h"

// 代码回退标志位

// 底盘控制任务内部使用的 PID 初始化函数。
static void chassis_pid_init(void);

// 4 个底盘驱动电机的转速闭环 PID，用于让每个车轮稳定跟踪目标转速。
PID pid_chassis_0, pid_chassis_1, pid_chassis_2, pid_chassis_3;
// 底盘平动/姿态相关 PID：
// - linear_x / linear_y：将速度误差转换为底盘 x、y 方向驱动力
// - angle：预留给底盘角度闭环控制
PID pid_F_chassis_linear_x, pid_F_chassis_angle, pid_F_chassis_linear_y;
// 航向角 PID，用于结合 IMU yaw 实现底盘定向或航向保持。
PID pid_yaw;
float rpm;
extern "C" void chassis_task(void *argument)
{
    // 初始化底盘任务依赖的 CAN、串口以及高精度计时模块。
    BSP_CAN::Init();
    BSP_USART::Init();
    DWT_.init(480);
    VescMotors[0].init(&hfdcan2, 80);

    // 配置相关 GPIO 电平，完成底盘板级外设的上电或使能控制。
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);

    // 初始化底盘控制链路中全部 PID 控制器。
    chassis_pid_init();
    osDelay(200);

   
    while (1) {
        // 1. 更新遥控器/离线保护等状态，并刷新底盘控制指令。
        remove_dji.monitor();
        remove_dji.updateChassosCommand();

        // 2. 读取 4 个底盘电机的实时转速，为后续底盘状态解算提供输入。
        for (uint8_t i = 0; i < 4; i++) {
            omni_chassis.now.rpm[i] = chassis_motor.Chassis_Motor[i].Data.Rpm;
        }
        // 3. 正运动学解算：根据各轮转速反推出底盘当前线速度等状态量。
        omni_chassis.forwardKinematics();

        float VZ_OUT = 0.0f;
        // 4. 遥控器输入处理：
        //    当拨杆处于指定档位时，启用航向保持控制，通过 yaw PID 输出底盘自转角速度。
        // if (remove_dji.rc_.s[0] == 3 || remove_dji.rc_.s[0] == 1) {
        //     VZ_OUT = pid_yaw.PID_Calculate(0.0f, dm_imu.imu.yaw);
        // }

        // 5. 生成底盘目标速度：
        //    - x 方向速度直接使用上层输入
        //    - y 方向速度经过 lift_auto 处理，用于与升降机构动作联动
        float target_vx = -remove_dji.chassis_.Vx;
        float target_vy = lift_auto.getChassisVyTarget(-remove_dji.chassis_.Vy);
        omni_chassis.setRemote(target_vx, target_vy, remove_dji.chassis_.Vz);

        // 6. 速度控制 PID：
        //    根据当前速度与目标速度误差，计算底盘在 x/y 方向所需的驱动力。
        float Fx = pid_F_chassis_linear_x.PID_Calculate(omni_chassis.now.Vx, omni_chassis.target.Vx);
        float Fy = pid_F_chassis_linear_y.PID_Calculate(omni_chassis.now.Vy, omni_chassis.target.Vy);

        // 7. 逆动力学：将底盘所需的合力分配为各个车轮应承担的输出。
        omni_chassis.dynamicsInverse(Fx, Fy, 0.0f);
        // 8. 逆运动学：将底盘目标运动量进一步转换成每个电机的目标转速。
        omni_chassis.inverseKinematics();

        // 9. 电机转速 PID：
        //    分别对 4 个底盘驱动电机进行转速闭环计算。
        pid_chassis_0.PID_Calculate(chassis_motor.Chassis_Motor[0].Data.Rpm, omni_chassis.target.rpm[0]);
        pid_chassis_1.PID_Calculate(chassis_motor.Chassis_Motor[1].Data.Rpm, omni_chassis.target.rpm[1]);
        pid_chassis_2.PID_Calculate(chassis_motor.Chassis_Motor[2].Data.Rpm, omni_chassis.target.rpm[2]);
        pid_chassis_3.PID_Calculate(chassis_motor.Chassis_Motor[3].Data.Rpm, omni_chassis.target.rpm[3]);

        float motor_input[4];
        // 10. 将转速环 PID 输出与模型计算得到的前馈量叠加，形成最终电机电流指令。
        motor_input[0] = pid_chassis_0.pid.Output + omni_chassis.Current_rpm[0];
        motor_input[1] = pid_chassis_1.pid.Output + omni_chassis.Current_rpm[1];
        motor_input[2] = pid_chassis_2.pid.Output + omni_chassis.Current_rpm[2];
        motor_input[3] = pid_chassis_3.pid.Output + omni_chassis.Current_rpm[3];

        // 11. 通过 CAN 总线发送电机电流：
        //     第一帧发送 4 个底盘驱动轮的控制电流；
        //     第二帧发送其余挂载电机的固定控制指令。
        chassis_motor.Send_CurrentCommand(&BSP_CAN::FDCAN3_TxFrame, 0x200, motor_input[0], motor_input[1], motor_input[2], motor_input[3]);
       // chassis_motor.Send_CurrentCommand(&BSP_CAN::FDCAN2_TxFrame, 0x200, 3000, 0, 0, 0);
      
            
           VescMotors[0].setRpm(rpm);
         //VescMotors[0].setHandbrakeCurrent(3000);

        // 12. 任务周期延时，维持底盘控制循环频率。
        osDelay(1);
    }
}

static void chassis_pid_init(void)
{
    // 4 个车轮各自的转速环 PID，负责底盘电机最基础的闭环稳速控制。
    pid_chassis_0.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);
    pid_chassis_1.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);
    pid_chassis_2.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);
    pid_chassis_3.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);

    // x 方向线速度 PID：将底盘 x 方向速度误差转换为驱动力 Fx。
    pid_F_chassis_linear_x.Init(OUTPUT_CHASSIS_LINEAR, INTERLIMIT_CHASSIS_LINEAR, DEBAND_CHASSIS_LINEAR, KP_CHASSIS_LINEAR, KI_CHASSIS_LINEAR, KD_CHASSIS_LINEAR, 0, 0x00);
    // y 方向线速度 PID：将底盘 y 方向速度误差转换为驱动力 Fy。
    pid_F_chassis_linear_y.Init(OUTPUT_CHASSIS_LINEAR, INTERLIMIT_CHASSIS_LINEAR, DEBAND_CHASSIS_LINEAR, KP_CHASSIS_LINEAR, KI_CHASSIS_LINEAR, KD_CHASSIS_LINEAR, 0, 0x00);
    // 底盘角度 PID：用于姿态或转角闭环控制，当前文件中暂未直接参与主循环计算。
    pid_F_chassis_angle.Init(OUTPUT_CHASSIS_ANGLE, INTERLIMIT_CHASSIS_ANGLE, DEBAND_CHASSIS_ANGLE, KP_CHASSIS_ANGLE, KI_CHASSIS_ANGLE, KD_CHASSIS_ANGLE, 0, 0x00);

    // 航向角保持 PID：根据 IMU yaw 偏差输出底盘自转控制量。
    pid_yaw.Init(2, 0, 0, 2, 0, 0, 0, 0x00);
}
