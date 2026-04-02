#include "lift_class.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "bsp_can.h"
#include "dji_motor.h"
#include "bsp_remove.h"
#include "bsp_dwt.h"
#include "bsp_usart.h"
#include "PID.h"
#include "yun_j60.h"
#include "lift_auto.h"

// 根据 2006 电机反馈转速，换算当前升降轮的线速度
static void calc_linear_speed_from_motor_rpm(void);
// 初始化升降机构相关 PID
static void lift_class_pid_init(void);
// 计算升降机构角度环与升降轮速度环 PID 输出
static void lift_class_pid_calculate(void);
// 根据云深电机当前位置，换算当前升降高度
static void lift_cauclate_height(void);
// 设置一次线性高度轨迹的起点、终点和总时间
static void lift_height_set_target(LiftHeight_t *lift, float target, float T);
// 将目标高度换算为云深电机目标角度
static float lift_position_input(float height);
// 按时间推进一次线性高度轨迹，并输出当前轨迹高度
static float lift_height_input(LiftHeight_t *lift);
// 根据目标线速度，反算升降轮对应的电机目标转速
static void calc_motor_rpm_from_linear_speed_target(float linear_speed_target);

// 升降角度控制 PID
PID pid_lift_left, pid_lift_right;
// 升降轮速度控制 PID
PID pid_2006_r, pid_2006_l;

// 升降机构实时状态
Lift_Class lift_class;
// 一次线性高度轨迹状态
LiftHeight_t lift_calulate = {0};
// 调试数据缓存
debug_lift lift_debug = {0};

extern "C" void lift_task(void *argument)
{
    // 依次使能 2 个云深电机
    yun_j60_motor.EnableMotor(0x02);
    yun_j60_motor.EnableMotor(0x03);

    // 初始化所有 PID 参数
    lift_class_pid_init();

    // 上电后先给一个回到 0 高度的初始轨迹
    lift_height_set_target(&lift_calulate, 0.0f, 2.0f);

    for (;;) {
        // 读取当前机械位置并换算为左右两侧高度
        lift_cauclate_height();

        // 读取 2006 电机反馈，换算当前升降轮的线速度
        calc_linear_speed_from_motor_rpm();

        // 更新半自动上台阶状态机
        lift_auto.update();

        // 根据手动或半自动给出的目标线速度，换算升降轮目标转速
        calc_motor_rpm_from_linear_speed_target(
            lift_auto.getLiftLinearSpeedTarget(remove_dji.chassis_.Vl));

        // 记录上一拍的档位，只在档位变化时重新生成高度目标
        static uint8_t last_sw = 0;

        // 如果半自动接管，则这里返回自动档位；否则返回手动拨杆档位
        uint8_t now_sw = lift_auto.getLiftSwitch(remove_dji.rc_.s[0]);

        // 只有档位变化时，才重新设置目标高度
        if (now_sw != last_sw) {
            if (now_sw == 3U) {
                // 3 档对应目标高度
                lift_debug.height_target = 0.0f;
                lift_debug.flag          = 1.0f;
            } else if (now_sw == 1U) {
                // 1 档对应目标高度
                lift_debug.height_target = 50.0f;
                lift_debug.flag          = 1.0f;
            } else if (now_sw == 2U) {
                // 2 档对应目标高度
                lift_debug.height_target = -205.0f;
                lift_debug.flag          = 1.0f;
            }

            // 更新上一拍档位
            last_sw = now_sw;
        }

        // 当标志位置位时，重新生成一条 0.7s 的线性轨迹
        if (lift_debug.flag == 1.0f) {
            lift_height_set_target(&lift_calulate, lift_debug.height_target, 0.7f);
            lift_debug.flag = 0.0f;
        }

        // 持续推进轨迹，得到当前应该跟踪的高度
        lift_debug.height_calulate = lift_height_input(&lift_calulate);

        // 将轨迹高度换算成云深电机的目标角度
        lift_debug.posi = lift_position_input(lift_debug.height_calulate);

        // 计算角度环和升降轮速度环 PID
        lift_class_pid_calculate();

   

        // 左右两侧云深电机按目标角度与 PID 力矩输出控制
        yun_j60_motor.SendControl(0x02, -lift_debug.posi, 0, 15, 0.5f, -pid_lift_left.pid.Output);
        yun_j60_motor.SendControl(0x03, lift_debug.posi, 0, 15, 0.5f, pid_lift_right.pid.Output);

        //2006 电机电流控制发送口目前保留
        lift_motor.Send_CurrentCommand(&BSP_CAN::FDCAN3_TxFrame,
                                        0x1FF,
                                        pid_2006_r.pid.Output,
                                        -pid_2006_l.pid.Output,
                                        0,
                                        0);

        osDelay(1);
    }
}

static void lift_class_pid_init(void)
{
    // 初始化左右两侧 2006 升降轮速度环 PID
    pid_2006_l.Init(OUTPUT_LIFT_MOVE,
                    INTERLIMIT_LIFT_MOVE,
                    DEBAND_LIFT_MOVE,
                    KP_LIFT_MOVE,
                    KI_LIFT_MOVE,
                    KD_LIFT_MOVE,
                    0,
                    0x00);

    pid_2006_r.Init(OUTPUT_LIFT_MOVE,
                    INTERLIMIT_LIFT_MOVE,
                    DEBAND_LIFT_MOVE,
                    KP_LIFT_MOVE,
                    KI_LIFT_MOVE,
                    KD_LIFT_MOVE,
                    0,
                    0x00);

    // 初始化左右两侧云深电机高度控制 PID
    pid_lift_left.Init(OUTPUT_LIFT,
                       INTERLIMIT_LIFT,
                       DEBAND_LIFT,
                       KP_LIFT,
                       KI_LIFT,
                       KD_LIFT,
                       0,
                       0x00);

    pid_lift_right.Init(OUTPUT_LIFT,
                        INTERLIMIT_LIFT,
                        DEBAND_LIFT,
                        KP_LIFT,
                        KI_LIFT,
                        KD_LIFT,
                        0,
                        0x00);
}

static void lift_class_pid_calculate(void)
{
    // 左右两侧高度环：当前高度 -> 轨迹高度
    pid_lift_left.PID_Calculate(lift_class.left.height, lift_debug.height_calulate);
    pid_lift_right.PID_Calculate(lift_class.right.height, lift_debug.height_calulate);

    // 左右两侧升降轮速度环：当前反馈 rpm -> 目标 rpm
    pid_2006_l.PID_Calculate(-lift_class.now.rpm_left, lift_class.target.rpm_left);
    pid_2006_r.PID_Calculate(lift_class.now.rpm_right, lift_class.target.rpm_right);
}

static void lift_cauclate_height(void)
{
    // 左侧电机角度方向与机械正方向相反，因此这里取负号
    lift_class.left.angle = -yun_j60_motor.motor[1].position;
    // 由卷轮直径换算左侧当前高度
    lift_class.left.height = lift_class.left.angle * HEIGHT_DIAMETER / 2.0f;

    // 右侧电机角度直接使用正方向
    lift_class.right.angle = yun_j60_motor.motor[2].position;
    // 由卷轮直径换算右侧当前高度
    lift_class.right.height = lift_class.right.angle * HEIGHT_DIAMETER / 2.0f;
}

static float lift_position_input(float height)
{
    // 对目标高度做限幅，避免超出机构行程
    if (height > 120.0f) {
        height = 120.0f;
    } else if (height < -220.0f) {
        height = -220.0f;
    }

    // 将线高度换算为卷轮需要转过的弧度
    return (height * 2.0f) / HEIGHT_DIAMETER;
}

static void lift_height_set_target(LiftHeight_t *lift, float target, float T)
{
    // 空指针保护
    if (lift == 0) {
        return;
    }

    // 保存轨迹起点，并写入本次目标和轨迹时间
    lift->start_height  = lift->current_height;
    lift->target_height = target;
    lift->total_time    = T;
    lift->start_time    = DWT_.getTimeline_s();
    lift->started       = 1;
    lift->finished      = 0;

    // 如果给出的轨迹时间小于等于 0，则直接瞬时到目标值
    if (T <= 0.0f) {
        lift->current_height = target;
        lift->finished       = 1;
    }
}

static float lift_height_input(LiftHeight_t *lift)
{
    float t;

    // 空指针保护
    if (lift == 0) {
        return 0.0f;
    }

    // 如果轨迹还没启动，直接返回当前高度
    if (lift->started == 0U) {
        return lift->current_height;
    }

    // 如果轨迹已经完成，直接返回目标高度
    if (lift->finished != 0U) {
        return lift->target_height;
    }

    // 计算从轨迹开始到现在已经过去的时间
    t = DWT_.getTimeline_s() - lift->start_time;

    if (t <= 0.0f) {
        // 时间未正式开始时，保持起点高度
        lift->current_height = lift->start_height;
    } else if (t >= lift->total_time) {
        // 超过总时间后，直接到终点并标记完成
        lift->current_height = lift->target_height;
        lift->finished       = 1U;
    } else {
        // 在总时间内按线性插值平滑推进高度轨迹
        lift->current_height = lift->start_height +
                               (lift->target_height - lift->start_height) * (t / lift->total_time);
    }

    return lift->current_height;
}

static void calc_linear_speed_from_motor_rpm(void)
{
    // 轮径从 mm 转成 m
    float wheel_diameter_m = WHEEL_DIAMETERr_MM / 1000.0f;

    // 根据齿轮齿数换算减速比
    float gear_ratio = (float)Z_MOTOR / (float)Z_WHEEL;

    // 当前电机反馈 rpm
    lift_class.now.rpm_left  = lift_motor.Lift_2006[1].Data.Rpm;
    lift_class.now.rpm_right = lift_motor.Lift_2006[0].Data.Rpm;

    // 电机 rpm -> 轮端 rpm
    float wheel_rpm_left  = lift_class.now.rpm_left * gear_ratio;
    float wheel_rpm_right = lift_class.now.rpm_right * gear_ratio;

    // 轮端 rpm -> 线速度 m/s
    float wheel_speed_left  = wheel_rpm_left * 3.1415926f * wheel_diameter_m / 60.0f;
    float wheel_speed_right = wheel_rpm_right * 3.1415926f * wheel_diameter_m / 60.0f;

    // 左右两侧按机械安装方向写入符号
    lift_class.now.vel_2006_left  = wheel_speed_left;
    lift_class.now.vel_2006_right = wheel_speed_right;
}

static void calc_motor_rpm_from_linear_speed_target(float linear_speed_target)
{
    // 轮径从 mm 转成 m
    float wheel_diameter_m = WHEEL_DIAMETERr_MM / 1000.0f;

    // 根据齿轮齿数换算减速比
    float gear_ratio = (float)Z_MOTOR / (float)Z_WHEEL;

    // 左右升降轮目标线速度方向相反
    float v_left  = linear_speed_target;
    float v_right = linear_speed_target;

    // 保存目标线速度
    lift_class.target.vel_2006_left  = v_left;
    lift_class.target.vel_2006_right = v_right;

    // 由目标线速度反算左右轮端 rpm
    float wheel_rpm_left  = (v_left) * 60.0f / (3.1415926f * wheel_diameter_m);
    float wheel_rpm_right = (v_right) * 60.0f / (3.1415926f * wheel_diameter_m);

    // 再由轮端 rpm 反算电机目标 rpm
    lift_class.target.rpm_left  = wheel_rpm_left / gear_ratio;
    lift_class.target.rpm_right = wheel_rpm_right / gear_ratio;
}