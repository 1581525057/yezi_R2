#ifndef ARM_COMM_H
#define ARM_COMM_H

#include <stdint.h>

// 机械臂通信命令类。
// 当前协议为固定 7 字节二进制帧：AA boot pick_count kfs_height zone3_cmd r2r1_fused 55。
class ArmComm
{
public:
       enum
    {
        FRAME_LENGTH = 7,
        RX_FRAME_LENGTH = 6
    };

    // 机械臂回传状态：[BB] [event] [sys_mode] [arm_kfs] [car_kfs] [EE]。
    struct RxData
    {
        uint8_t event;    // 1=吸成功，2=存/放成功，3=可接受下一个指令，4=保护/异常
        uint8_t sys_mode; // 0=未开机，1=启动成功
        uint8_t arm_kfs;  // 0=手上没有，1=手上吸着一个
        uint8_t car_kfs;  // 车内 KFS 数量：0/1/2
    };

    // 动作代号：调用 executeAction() 输入这些代号即可生成对应帧。
    enum ActionCode
    {
        ACTION_POWER_ON_INIT = 1, // 开机初始化
        ACTION_PICK_HIGH_200,     // 取高 200mm KFS
        ACTION_PICK_HIGH_400,     // 取高 400mm KFS
        ACTION_PICK_LOW_200,      // 取矮 200mm KFS
        ACTION_ZONE3_READY,       // 去九宫格预备位
        ACTION_ZONE3_PLACE_HAND,  // 放手持 KFS
        ACTION_ZONE3_FETCH_UPPER, // 取车上层 KFS
        ACTION_ZONE3_PLACE_UPPER, // 放车上层 KFS
        ACTION_ZONE3_FETCH_LOWER, // 取车底层 KFS
        ACTION_ZONE3_RESET,       // 重置状态机，zone3_cmd 回 0
        ACTION_POWER_OFF,         // 关机
        ACTION_PICK_FIRST_KFS,    // 拾取第一个 KFS
        ACTION_PICK_SECOND_KFS,   // 拾取第二个 KFS
    };

    ArmComm();

    // 清空帧缓存为关机帧。
    void reset(void);

    // 根据动作代号生成帧。num_KFS 用作取物触发值，返回 1 表示成功，0 表示未知动作代号。
    uint8_t executeAction(uint8_t action_code, uint8_t num_KFS);

    // 返回当前 7 字节帧缓存，调用方可直接按 getFrameLength() 长度发送。
    const uint8_t *getFrame(void) const;

    // 返回固定帧长度，避免调用方手写魔法数字。
    uint8_t getFrameLength(void) const;

    // 串口发送入口。串口实现未接入前保持空函数。
    void send(void);

    // 解析机械臂 6 字节回传帧，返回 1 表示成功，0 表示帧格式错误。
    uint8_t parseRxFrame(const uint8_t *data, uint8_t length);

    // 获取最近一次成功解析的机械臂回传状态。
    const RxData &getRxData(void) const;

    // 周期调用取 KFS：未上台阶按 0 度沿当前 X 轴走 PICK_KFS_BEFORE_STEP_ADVANCE_CM；
    // 已上台阶以当前方块中心为基准，按当前 yaw 方向走 PICK_KFS_AFTER_STEP_ADVANCE_CM。
    uint8_t pickKFS(uint8_t action_code,
                    uint8_t num_KFS,
                    uint8_t already_step_up,
                    float current_x_m,
                    float current_y_m,
                    float yaw_deg,
                    uint8_t finish_at_center = 0U,
                    float center_x_m = 0.0f,
                    float center_y_m = 0.0f);

    // 取 KFS 流程接管底盘时返回内部目标速度，否则透传 manual_target。
    float getChassisVxTarget(float manual_target) const;
    float getChassisVyTarget(float manual_target) const;

    // 外部流程复位时清空取 KFS 状态，不清空机械臂回传数据。
    void resetPickKFS(void);

    RxData rx_data_;

private:
    enum FrameIndex
    {
        INDEX_HEAD = 0,
        INDEX_BOOT,
        INDEX_PICK_COUNT,
        INDEX_KFS_HEIGHT,
        INDEX_ZONE3_CMD,
        INDEX_R2R1_FUSED,
        INDEX_TAIL,
    };

    enum RxFrameIndex
    {
        RX_INDEX_HEAD = 0,
        RX_INDEX_EVENT,
        RX_INDEX_SYS_MODE,
        RX_INDEX_ARM_KFS,
        RX_INDEX_CAR_KFS,
        RX_INDEX_TAIL,
    };

    enum PickKFSState
    {
        PICK_KFS_IDLE = 0,
        PICK_KFS_MOVE,
        PICK_KFS_SEND,
        PICK_KFS_WAIT_DONE,
        PICK_KFS_RETURN_CENTER,
    };

    uint8_t frame_[FRAME_LENGTH];
    PickKFSState pick_kfs_state_;
    uint8_t pick_kfs_action_code_;
    uint8_t pick_kfs_num_;
    uint8_t pick_kfs_stable_count_;
    float pick_kfs_target_x_m_;
    float pick_kfs_target_y_m_;
    float pick_kfs_vx_target_;
    float pick_kfs_vy_target_;
    uint8_t pick_kfs_zero_yaw_move_;

    // 统一写帧，帧头帧尾固定，动作只填中间 5 个协议字节。
    void setFrame(uint8_t boot,
                  uint8_t pick_count,
                  uint8_t kfs_height,
                  uint8_t zone3_cmd,
                  uint8_t r2r1_fused);

    float speed_limit(float speed, float max) const;
    float trapezoid_speed(float error, float acc, float max) const;
    uint8_t pick_kfs_stable_confirm(uint8_t condition);
};

extern ArmComm arm_comm;

#endif
