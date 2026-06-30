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
#include "lift_step_up.h"
#include "lift_step_down.h"
#include "VescMotor.h"
#include "mieling.h"
#include "usart_task.h"
#include "route_task.h"
#include "arm_comm.h"
#include "FTMTask.h"
#include "conbat_task.h"

extern "C" float WuqiquTask_GetChassisVxTarget(float manual);
extern "C" float WuqiquTask_GetChassisVyTarget(float manual);
extern "C" float WuqiquTask_GetChassisVzTarget(float manual);
extern "C" uint8_t WuqiquTask_IsActive(void);

typedef enum
{
    CHASSIS_AUTO_NONE = 0,
    CHASSIS_AUTO_MEILING,
    CHASSIS_AUTO_WUQIQU,
    CHASSIS_AUTO_ROUTE_PATH,
    CHASSIS_AUTO_CONBAT,
    CHASSIS_AUTO_CONFLICT
} ChassisAutoSource;

// 底盘自动控制只允许一个来源生效，避免多个任务同时改写目标速度。
static inline ChassisAutoSource ChassisAuto_SelectSource(uint8_t wuqiqu_active,
                                                         uint8_t meiling_area_active,
                                                         uint8_t route_active,
                                                         uint8_t conbat_active)
{
    uint8_t active_count = 0U;

    if (wuqiqu_active != 0U)
    {
        active_count++;
    }
    if (meiling_area_active != 0U)
    {
        active_count++;
    }
    if (route_active != 0U)
    {
        active_count++;
    }
    if (conbat_active != 0U)
    {
        active_count++;
    }

    if (active_count > 1U)
    {
        return CHASSIS_AUTO_CONFLICT;
    }
    if (wuqiqu_active != 0U)
    {
        return CHASSIS_AUTO_WUQIQU;
    }
    if (meiling_area_active != 0U)
    {
        return CHASSIS_AUTO_MEILING;
    }
    if (route_active != 0U)
    {
        return CHASSIS_AUTO_ROUTE_PATH;
    }
    if (conbat_active != 0U)
    {
        return CHASSIS_AUTO_CONBAT;
    }
    return CHASSIS_AUTO_NONE;
}

// route_task 转弯阶段由全局 yaw_target 接管底盘角度。
static inline uint8_t ChassisAuto_IsRouteYawTurnActive(void)
{
    if (route_t.flag_start != 1U)
    {
        return 0U;
    }

    return ((route_t.state == PHASE_TURN_LEFT90) ||
            (route_t.state == PHASE_TURN_RIGHT90) ||
            (route_t.state == PHASE_TURN180))
               ? 1U
               : 0U;
}

// 初始化底盘任务中使用的全部 PID。
static void chassis_pid_init(void);

// 4 个底盘驱动电机的转速闭环 PID。
PID pid_chassis_0,
    pid_chassis_1, pid_chassis_2, pid_chassis_3;

// 底盘速度环 PID：将 x/y 方向速度误差转换为底盘驱动力。
PID pid_F_chassis_linear_x, pid_F_chassis_angle, pid_F_chassis_linear_y;

// 航向保持 PID：根据视觉角度与目标角度的偏差输出自转速度。
PID pid_yaw;
static float target_vx_last = 0.0f;
static float target_vy_last = 0.0f;
static float target_vz_last = 0.0f;
float rpm, current;
float yaw_target;
float x_M;
extern "C" void chassis_task(void *argument)
{
    // 初始化底盘任务依赖的通信、计时和控制器模块。
    BSP_CAN::Init();
    BSP_USART::Init();
    DWT_.init(480);
    chassis_pid_init();

    VescMotors[0].init(&hfdcan2, 91);
    VescMotors[1].init(&hfdcan2, 103);
    VescMotors[2].init(&hfdcan2, 121);
    VescMotors[3].init(&hfdcan2, 80);

    // 使能底盘相关板级外设。
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);

    osDelay(200);

    while (1)
    {

        // 更新遥控器状态和底盘人工控制指令。
        remove_dji.monitor();
        remove_dji.updateChassosCommand();

        // 读取 4 个 VESC 电机的实际转速，作为底盘状态解算输入。
        for (uint8_t i = 0; i < 4; i++)
        {
            omni_chassis.now.rpm[i] = VescMotors[i].rxData_.rpm;
        }

        // 正运动学：由轮速反推底盘当前速度。
        omni_chassis.forwardKinematics();

        // 先取遥控器目标速度；自动任务生效时会覆盖对应目标。
        float target_vx = remove_dji.chassis_.Vx;
        float target_vy = remove_dji.chassis_.Vy;
        float target_vz = remove_dji.chassis_.Vz;
        float route_path_vx = target_vx;
        float route_path_vy = target_vy;
        float route_path_vz = target_vz;
        uint8_t route_path_active = route_t.getPathChassisTarget(target_vx,
                                                                 target_vy,
                                                                 target_vz,
                                                                 &route_path_vx,
                                                                 &route_path_vy,
                                                                 &route_path_vz);
        const uint8_t route_yaw_turn_active = ChassisAuto_IsRouteYawTurnActive();
        const uint8_t route_auto_active = ((route_path_active != 0U) || (route_yaw_turn_active != 0U)) ? 1U : 0U;
        float conbat_vx = target_vx;
        float conbat_vy = target_vy;
        float conbat_vz = target_vz;
        const uint8_t conbat_active = conbat_t.getChassisTarget(target_vx,
                                                                target_vy,
                                                                target_vz,
                                                                &conbat_vx,
                                                                &conbat_vy,
                                                                &conbat_vz);

        const ChassisAutoSource auto_source = ChassisAuto_SelectSource(WuqiquTask_IsActive(),
                                                                       RouteTask_IsMeilingAreaActive(),
                                                                       route_auto_active,
                                                                       conbat_active);
        switch (auto_source)
        {
        case CHASSIS_AUTO_WUQIQU:
            // 武器区接管底盘速度。
            target_vx = WuqiquTask_GetChassisVxTarget(target_vx);
            target_vy = WuqiquTask_GetChassisVyTarget(target_vy);
            target_vz = WuqiquTask_GetChassisVzTarget(target_vz);
            break;
        case CHASSIS_AUTO_MEILING:
            // 梅林区接管底盘速度。
            target_vx = meiling.getChassisVxTarget(target_vx);
            target_vy = meiling.getChassisVyTarget(target_vy);
            target_vz = meiling.getChassisVzTarget(target_vz);
            break;
        case CHASSIS_AUTO_ROUTE_PATH:
            // 1 区跑点接管底盘速度。
            target_vx = route_path_vx;
            target_vy = route_path_vy;
            target_vz = route_path_vz;
            break;
        case CHASSIS_AUTO_CONBAT:
            // conbat_task 接管底盘速度，状态占位时默认输出零速度。
            target_vx = conbat_vx;
            target_vy = conbat_vy;
            target_vz = conbat_vz;
            break;
        case CHASSIS_AUTO_CONFLICT:
            // 两个自动任务同时生效时停车，避免底盘控制权冲突。
            target_vx = 0.0f;
            target_vy = 0.0f;
            target_vz = 0.0f;
            break;
        default:
            break;
        }

        if (FTM_GetMainState() == 4U)
        {
            target_vz = -pid_yaw.PID_Calculate_Angle(vision.angle_x, yaw_target);
        }
        else if (FTM_IsYawTargetCorrectionEnabled() != 0U)
        {
            if (FTM_IsYawTargetTurnActive() != 0U)
            {
                target_vx = 0.0f;
                target_vy = 0.0f;
            }

            target_vz = -pid_yaw.PID_Calculate_Angle(vision.angle_x, FTM_GetYawTargetDegree());
        }
        else if (auto_source == CHASSIS_AUTO_CONBAT && conbat_t.yaw_target_enabled != 0U)
        {
            target_vz = -pid_yaw.PID_Calculate_Angle(vision.angle_x, conbat_t.yaw_target_degree);
        }

        if (auto_source == CHASSIS_AUTO_ROUTE_PATH && route_yaw_turn_active != 0U)
        {
            target_vx = 0.0f;
            target_vy = 0.0f;
            // route_task 已经写入 yaw_target，这里只负责按全局目标转向。
            target_vz = -pid_yaw.PID_Calculate_Angle(vision.angle_x, yaw_target);
        }

        // 抬升机构根据自身状态继续限制底盘平移速度。
        target_vy = lift_auto.getChassisVyTarget(target_vy);
        target_vx = lift_auto.getChassisVxTarget(target_vx);
        target_vy = lift_step_down.getChassisVyTarget(target_vy);
        target_vx = lift_step_down.getChassisVxTarget(target_vx);
        target_vy = arm_comm.getChassisVyTarget(target_vy);
        target_vx = arm_comm.getChassisVxTarget(target_vx);

        // 自转输出统一走 target_vz：默认遥控器 Vz，FTM_MAIN_DONE 锁角和自动任务可覆盖。
        omni_chassis.setRemote(target_vx, target_vy, target_vz);

        // 速度环：根据当前速度和目标速度的误差计算 x/y 方向驱动力。
        float Fx = pid_F_chassis_linear_x.PID_Calculate(omni_chassis.now.Vx, omni_chassis.target.Vx);
        float Fy = pid_F_chassis_linear_y.PID_Calculate(omni_chassis.now.Vy, omni_chassis.target.Vy);

        // 逆动力学将底盘驱动力分配到各个车轮。
        omni_chassis.dynamicsInverse(Fx, Fy, 0.0f);

        // 逆运动学将底盘目标速度转换为各电机目标转速。
        omni_chassis.inverseKinematics();

        // 电机速度环：每个 VESC 电机独立跟踪对应目标转速。
        pid_chassis_0.PID_Calculate(VescMotors[0].rxData_.rpm, omni_chassis.target.rpm[0]);
        pid_chassis_1.PID_Calculate(VescMotors[1].rxData_.rpm, omni_chassis.target.rpm[1]);
        pid_chassis_2.PID_Calculate(VescMotors[2].rxData_.rpm, omni_chassis.target.rpm[2]);
        pid_chassis_3.PID_Calculate(VescMotors[3].rxData_.rpm, omni_chassis.target.rpm[3]);

        float motor_input[4];

        // 通过 CAN 分别发送前馈电流和转速闭环输出。
        for (uint8_t i = 0; i < 4; i++)
        {

            motor_input[i] = omni_chassis.target.rpm[i];
            VescMotors[i].setRpm(motor_input[i]);
        }

        // 控制周期约 1 ms。
        osDelay(1);
    }
}

static void chassis_pid_init(void)
{
    // 车轮转速环 PID。
    pid_chassis_0.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);
    pid_chassis_1.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);
    pid_chassis_2.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);
    pid_chassis_3.Init(OUTPUT_CHASSIS_3508, INTERLIMIT_CHASSIS_3508, DEBAND_CHASSIS_3508, KP_CHASSIS_3508, KI_CHASSIS_3508, KD_CHASSIS_3508, 0, 0x00);

    // 底盘 x/y 方向速度环 PID。
    pid_F_chassis_linear_x.Init(OUTPUT_CHASSIS_LINEAR, INTERLIMIT_CHASSIS_LINEAR, DEBAND_CHASSIS_LINEAR, KP_CHASSIS_LINEAR, KI_CHASSIS_LINEAR, KD_CHASSIS_LINEAR, 0, 0x00);
    pid_F_chassis_linear_y.Init(OUTPUT_CHASSIS_LINEAR, INTERLIMIT_CHASSIS_LINEAR, DEBAND_CHASSIS_LINEAR, KP_CHASSIS_LINEAR, KI_CHASSIS_LINEAR, KD_CHASSIS_LINEAR, 0, 0x00);

    // 航向保持 PID。
    pid_yaw.Init(3.5, 0.8, 0.5, 0.08, 0.1, 0.01, 0, 0x00);
}
