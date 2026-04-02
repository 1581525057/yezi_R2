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

static void chassis_pid_init(void);

PID pid_chassis_0, pid_chassis_1, pid_chassis_2, pid_chassis_3;
PID pid_F_chassis_linear_x, pid_F_chassis_angle, pid_F_chassis_linear_y;
PID pid_yaw;

extern "C" void chassis_task(void *argument)
{
    BSP_CAN::Init();
    BSP_USART::Init();
    DWT_.init(480);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);

    chassis_pid_init();
    while (1) {
        remove_dji.monitor();
        remove_dji.updateChassosCommand();

        for (uint8_t i = 0; i < 4; i++) {
            omni_chassis.now.rpm[i] = chassis_motor.Chassis_Motor[i].Data.Rpm;
        }
        omni_chassis.forwardKinematics();

        float VZ_OUT = 0.0f;
        if (remove_dji.rc_.s[0] == 3 || remove_dji.rc_.s[0] == 1) {
            VZ_OUT = pid_yaw.PID_Calculate(0.0f, dm_imu.imu.yaw);
        }

        float target_vx = -remove_dji.chassis_.Vx;
        float target_vy = lift_auto.getChassisVyTarget(-remove_dji.chassis_.Vy);
        omni_chassis.setRemote(target_vx, target_vy, VZ_OUT);

        float Fx = pid_F_chassis_linear_x.PID_Calculate(omni_chassis.now.Vx, omni_chassis.target.Vx);
        float Fy = pid_F_chassis_linear_y.PID_Calculate(omni_chassis.now.Vy, omni_chassis.target.Vy);

        omni_chassis.dynamicsInverse(Fx, Fy, 0.0f);
        omni_chassis.inverseKinematics();

        pid_chassis_0.PID_Calculate(chassis_motor.Chassis_Motor[0].Data.Rpm, omni_chassis.target.rpm[0]);
        pid_chassis_1.PID_Calculate(chassis_motor.Chassis_Motor[1].Data.Rpm, omni_chassis.target.rpm[1]);
        pid_chassis_2.PID_Calculate(chassis_motor.Chassis_Motor[2].Data.Rpm, omni_chassis.target.rpm[2]);
        pid_chassis_3.PID_Calculate(chassis_motor.Chassis_Motor[3].Data.Rpm, omni_chassis.target.rpm[3]);

        float motor_input[4];
        motor_input[0] = pid_chassis_0.pid.Output + omni_chassis.Current_rpm[0];
        motor_input[1] = pid_chassis_1.pid.Output + omni_chassis.Current_rpm[1];
        motor_input[2] = pid_chassis_2.pid.Output + omni_chassis.Current_rpm[2];
        motor_input[3] = pid_chassis_3.pid.Output + omni_chassis.Current_rpm[3];

         chassis_motor.Send_CurrentCommand(&BSP_CAN::FDCAN3_TxFrame, 0x200, motor_input[0], motor_input[1], motor_input[2], motor_input[3]);
		chassis_motor.Send_CurrentCommand(&BSP_CAN::FDCAN2_TxFrame, 0x200,3000,0,0,0);
        osDelay(1);
    }
}

static void chassis_pid_init(void)
{
    pid_chassis_0.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);
    pid_chassis_1.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);
    pid_chassis_2.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);
    pid_chassis_3.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);

    pid_F_chassis_linear_x.Init(OUTPUT_CHASSIS_LINEAR, INTERLIMIT_CHASSIS_LINEAR, DEBAND_CHASSIS_LINEAR, KP_CHASSIS_LINEAR, KI_CHASSIS_LINEAR, KD_CHASSIS_LINEAR, 0, 0x00);
    pid_F_chassis_linear_y.Init(OUTPUT_CHASSIS_LINEAR, INTERLIMIT_CHASSIS_LINEAR, DEBAND_CHASSIS_LINEAR, KP_CHASSIS_LINEAR, KI_CHASSIS_LINEAR, KD_CHASSIS_LINEAR, 0, 0x00);
    pid_F_chassis_angle.Init(OUTPUT_CHASSIS_ANGLE, INTERLIMIT_CHASSIS_ANGLE, DEBAND_CHASSIS_ANGLE, KP_CHASSIS_ANGLE, KI_CHASSIS_ANGLE, KD_CHASSIS_ANGLE, 0, 0x00);

    pid_yaw.Init(2, 0, 0, 2, 0, 0, 0, 0x00);
}