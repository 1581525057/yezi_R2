/**
 * @file    VescMotor.hpp
 * @brief   VESC电调 CAN通信驱动 —— C++ 面向对象封装
 * @author  重构自 vesc_can.h / vesc_can.c
 * @date    2025
 *
 * ============================================================
 *  【原始C代码 → C++重构说明】
 * ============================================================
 *  原代码用函数指针模拟"成员方法"，本版本改为真正的C++类：
 *    - 数据成员 (private)  取代全局/结构体字段
 *    - 成员函数 (public)   取代函数指针 + 独立实现函数
 *    - 构造函数            取代 VESC_Motor_Struct_Init()
 *    - 无全局变量          tx_header 移入成员函数作用域
 *
 * ============================================================
 *  【STM32H7 FDCAN 外设基础知识】
 * ============================================================
 *  STM32H7 上的 FDCAN 外设与 F4/F1 上的 bxCAN 不同，主要区别：
 *
 *  1. 寄存器句柄：FDCAN_HandleTypeDef（bxCAN 是 CAN_HandleTypeDef）
 *     - hfdcan.Instance  指向 FDCAN1 / FDCAN2 / FDCAN3 基址
 *     - CubeMX 中可配置波特率、采样点、FD 模式（本驱动只用经典 CAN）
 *
 *  2. 发送流程（替代 HAL_CAN_AddTxMessage）：
 *     a. 填充 FDCAN_TxHeaderTypeDef
 *        .IdType      = FDCAN_EXTENDED_ID (29位) / FDCAN_STANDARD_ID (11位)
 *        .TxFrameType = FDCAN_DATA_FRAME  (数据帧) / FDCAN_REMOTE_FRAME
 *        .Identifier  = 29位扩展ID（VESC 协议使用扩展帧）
 *        .DataLength  = FDCAN_DLC_BYTES_8 (8字节，等价于 DLC=8)
 *                       其余字段（BitRateSwitch、FDFormat等）在经典CAN下
 *                       由 HAL 自动处理，无需手动配置
 *     b. 调用 HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data)
 *        - TxFifoQ：TX FIFO 队列模式，按顺序发送
 *        - 替代方案 TxBuffer：手动指定 buffer 号，更灵活但复杂
 *
 *  3. 接收流程（替代 HAL_CAN_GetRxMessage）：
 *     通常在中断回调中处理：
 *     void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
 *     {
 *         FDCAN_RxHeaderTypeDef rx_header;
 *         uint8_t rx_data[8];
 *         HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
 *         // rx_header.Identifier 含 29 位扩展 ID
 *     }
 *     本工程使用了自定义的 FIFO0_FDCAN_RxDATA 结构体缓存接收数据，
 *     由 bsp_can 层在中断中填充，本驱动直接读取该全局缓存。
 *
 *  4. 过滤器配置（CubeMX 或 HAL 手动配置）：
 *     FDCAN_FilterTypeDef filter;
 *     filter.IdType       = FDCAN_EXTENDED_ID;
 *     filter.FilterIndex  = 0;
 *     filter.FilterType   = FDCAN_FILTER_MASK;
 *     filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
 *     filter.FilterID1    = 0x00000000;  // 接受所有 ID
 *     filter.FilterID2    = 0x00000000;  // 掩码全0=全通
 *     HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
 *     HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
 *         FDCAN_ACCEPT_IN_RX_FIFO0,   // 非匹配扩展帧 → FIFO0
 *         FDCAN_ACCEPT_IN_RX_FIFO0,   // 非匹配标准帧 → FIFO0
 *         FDCAN_FILTER_REMOTE,        // 远程帧过滤
 *         FDCAN_FILTER_REMOTE);
 *     HAL_FDCAN_Start(&hfdcan1);
 *     HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
 *
 * ============================================================
 *  【VESC CAN 协议说明】
 * ============================================================
 *  VESC 使用 CAN 扩展帧（29位ID），ID 布局：
 *
 *    Bits[7:0]  = VESC 节点 ID（0~255，每个电调唯一）
 *    Bits[15:8] = 命令类型（CAN_PACKET_ID 枚举值）
 *    Bits[28:16]= 固定为 0（VESC 协议未使用高位）
 *
 *  所以发送一条命令：
 *    Identifier = (node_id & 0xFF) | ((uint32_t)cmd << 8)
 *
 *  接收时解析：
 *    node_id = Identifier & 0xFF
 *    cmd     = (Identifier >> 8) & 0xFF
 *
 * ============================================================
 *  【极对数与 RPM 换算】
 * ============================================================
 *  VESC 内部使用"电气转速 eRPM"，与机械转速的关系：
 *    eRPM = 机械 RPM × 极对数(pairs_of_poles)
 *  本项目电机：24N28P → 极对数 = 28/2 = 14
 *  发送 RPM 指令时需乘以 14 转换为 eRPM 再发送。
 *  接收状态帧时，eRPM 除以 14 得到机械转速。
 *
 * ============================================================
 */

#pragma once  // 等价于 #ifndef 头文件保护，更简洁

/* ============================================================
 *  头文件包含
 * ============================================================ */
#include "main.h"               // STM32 HAL 总头文件，包含 stm32h7xx_hal.h
#include "stm32h7xx_hal_fdcan.h"// FDCAN HAL 驱动（H7 专用，提供 FDCAN_HandleTypeDef）
#include "bsp_can.h"            // 板级 CAN 支持：FDCAN_RxFrame_TypeDef 等自定义类型

/* ============================================================
 *  外部 FDCAN 句柄声明
 *  这些句柄在 CubeMX 生成的 fdcan.c 中定义，此处声明引用
 * ============================================================ */
extern FDCAN_HandleTypeDef hfdcan1;  // FDCAN1 外设句柄
extern FDCAN_HandleTypeDef hfdcan2;  // FDCAN2 外设句柄
extern FDCAN_HandleTypeDef hfdcan3;  // FDCAN3 外设句柄

/* ============================================================
 *  BSP_CAN 层静态成员引用（不需要额外 extern）
 *  接收数据直接通过 BSP_CAN::FDCAN_RxFIFO0Frame / FDCAN_RxFIFO1Frame 访问，
 *  这两个静态成员在 bsp_can.cpp 中定义，bsp_can.h 中已声明。
 *  VescMotor::canRxHandler() 接收指向该结构体的指针，避免对全局命名空间的依赖。
 * ============================================================ */

/* ============================================================
 *  VESC CAN 命令类型枚举
 *  每个枚举值对应 CAN 扩展ID 的 Bits[15:8]，即"命令字"
 * ============================================================ */
enum class CanPacketID : uint8_t
{
    SET_DUTY                     = 0,   ///< 设置占空比（-1.0 ~ +1.0，协议内放大100000倍）
    SET_CURRENT                  = 1,   ///< 设置目标电流（单位 mA，协议内放大1000倍→μA）
    SET_CURRENT_BRAKE            = 2,   ///< 设置制动电流（单位 mA，协议内放大1000倍）
    SET_RPM                      = 3,   ///< 设置目标转速（eRPM，= 机械RPM × 极对数）
    SET_POS                      = 4,   ///< 设置目标位置（度，协议内放大1000000倍）

    FILL_RX_BUFFER               = 5,   ///< 填充接收缓冲区（多帧传输用）
    FILL_RX_BUFFER_LONG          = 6,   ///< 填充长缓冲区（多帧传输用）
    PROCESS_RX_BUFFER            = 7,   ///< 处理接收缓冲区数据（多帧结束标志）
    PROCESS_SHORT_BUFFER         = 8,   ///< 处理短缓冲区数据

    STATUS                       = 9,   ///< 状态帧1：eRPM、电流、占空比
    SET_CURRENT_REL              = 10,  ///< 相对电流（-1.0~1.0，对应最大电流百分比）
    SET_CURRENT_BRAKE_REL        = 11,  ///< 相对制动电流
    SET_CURRENT_HANDBRAKE        = 12,  ///< 手刹电流（绝对值，mA）
    SET_CURRENT_HANDBRAKE_REL    = 13,  ///< 手刹相对电流（0~1.0）
    STATUS_2                     = 14,  ///< 状态帧2：安时消耗 / 安时回充
    STATUS_3                     = 15,  ///< 状态帧3：瓦时消耗 / 瓦时回充
    STATUS_4                     = 16,  ///< 状态帧4：MOSFET温度、电机温度、PID位置
    PING                         = 17,  ///< Ping（探测在线）
    PONG                         = 18,  ///< Pong（在线应答）
    DETECT_APPLY_ALL_FOC         = 19,  ///< FOC 自动检测并应用
    DETECT_APPLY_ALL_FOC_RES     = 20,  ///< FOC 自动检测结果
    CONF_CURRENT_LIMITS          = 21,  ///< 配置电流限制
    CONF_STORE_CURRENT_LIMITS    = 22,  ///< 存储电流限制到 EEPROM
    CONF_CURRENT_LIMITS_IN       = 23,  ///< 配置输入电流限制
    CONF_STORE_CURRENT_LIMITS_IN = 24,  ///< 存储输入电流限制
    CONF_FOC_ERPMS               = 25,  ///< 配置 FOC eRPM 参数
    CONF_STORE_FOC_ERPMS         = 26,  ///< 存储 FOC eRPM 参数
    STATUS_5                     = 27,  ///< 状态帧5：输入电压、里程计数
};

/* ============================================================
 *  电机实时接收数据结构体
 *  由 CAN_Rx_Handler 解析后填入，可直接读取
 * ============================================================ */
struct VescRxData
{
    // ------- 状态帧1 (STATUS) 解析字段 -------
    float eRpm;             ///< 电气转速 (eRPM)，= 机械RPM × 极对数
    float rpm;              ///< 机械转速 (RPM)，= eRPM / 极对数
    float duty;             ///< 当前占空比，范围 -1.0 ~ +1.0
    float totalCurrent;     ///< 总相电流 (A)

    // ------- 状态帧4 (STATUS_4) 解析字段 -------
    float pidPositionNow;   ///< 当前 PID 角度 (度)，范围 0~360，VESC 以50倍压缩存储
    float pidPositionLast;  ///< 上一次 PID 角度，用于检测跨零点方向

    // ------- 累计位置计算（基于 STATUS_4） -------
    int   turnCount;        ///< 圈数计数器（正转+1，反转-1）
    float totalPosition;    ///< 累计角度 (度) = turnCount×360 + pidPositionNow

    // ------- 其他（STATUS 中可选） -------
    float dutyCycle;        ///< 占空比副本（与 duty 相同，保持兼容）

    // 默认构造清零
    VescRxData()
        : eRpm(0.f), rpm(0.f), duty(0.f), totalCurrent(0.f)
        , pidPositionNow(0.f), pidPositionLast(0.f)
        , turnCount(0), totalPosition(0.f), dutyCycle(0.f)
    {}
};

/* ============================================================
 *  VescMotor 类：封装单个 VESC 电调的全部操作
 *
 *  【使用方法】
 *  ① 声明对象（可放全局或局部）：
 *       VescMotor motor1;
 *
 *  ② 初始化（绑定 CAN 外设句柄和节点 ID）：
 *       motor1.init(&hfdcan2, 10);   // 使用 FDCAN2，VESC 节点ID=10
 *
 *  ③ 在主循环中调用控制接口：
 *       motor1.setCurrent(5000);      // 5000 mA
 *       motor1.setRpm(300);           // 300 RPM（内部自动×极对数）
 *       motor1.setPwm(0.5);           // 50% 占空比
 *
 *  ④ 在 CAN 接收中断（或回调）中更新状态：
 *       // HAL_FDCAN_RxFifo0Callback 内：
 *       uint8_t rx_data[8];
 *       // ... 获取数据 ...
 *       motor1.canRxHandler(&hfdcan2, rx_data);
 *
 *  ⑤ 读取反馈数据：
 *       float speed = motor1.getRxData().rpm;
 *       float angle = motor1.getRxData().totalPosition;
 * ============================================================ */
class VescMotor
{
public:
    /* --------------------------------------------------------
     *  构造函数：不绑定外设，需显式调用 init()
     * -------------------------------------------------------- */
    VescMotor();

    /* --------------------------------------------------------
     *  init() —— 替代原 VESC_Motor_Struct_Init()
     *
     *  @param hfdcan   指向 HAL FDCAN 句柄（&hfdcan1 / &hfdcan2 / &hfdcan3）
     *  @param nodeId   VESC 节点 CAN ID（0~255），需与 VESC Tool 中配置的 ID 一致
     *
     *  FDCAN 句柄在调用本函数前必须已经由 CubeMX 生成的 MX_FDCANx_Init() 初始化。
     *  本驱动不负责滤波器配置，请在 MX_FDCANx_Init() 后额外调用：
     *    HAL_FDCAN_ConfigFilter / HAL_FDCAN_ConfigGlobalFilter
     *    HAL_FDCAN_Start
     *    HAL_FDCAN_ActivateNotification(..., FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)
     * -------------------------------------------------------- */
    void init(FDCAN_HandleTypeDef* hfdcan, uint16_t nodeId);

    /* ========================================================
     *  控制接口（TX 方向：MCU → VESC）
     * ======================================================== */

    /**
     * @brief 设置电流（力矩控制模式）
     * @param current_mA  目标电流，单位 mA（毫安）
     *                    正值=正转驱动，负值=反转驱动
     *
     * 协议细节：
     *   VESC 接收的是 μA 精度，协议里 int32 = 电流mA × 1000
     *   → data[0..3] = big-endian int32(current_mA × 1000)
     *   CAN ID: bits[7:0]=nodeId, bits[15:8]=CAN_PACKET_SET_CURRENT(1)
     */
    void setCurrent(int32_t current_mA);

    /**
     * @brief 设置转速（速度控制模式）
     * @param rpm  目标机械转速，单位 RPM
     *             正值=正转，负值=反转
     *
     * 协议细节：
     *   VESC 使用 eRPM，内部自动换算：eRPM = rpm × pairs_of_poles
     *   → data[0..3] = big-endian int32(eRPM)
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_RPM(3)
     */
    void setRpm(int32_t rpm);

    /**
     * @brief 设置占空比（开环电压控制模式）
     * @param pwm  占空比，范围 -1.0（满反转）~ +1.0（满正转）
     *             0.0 = 停止（但不刹车）
     *
     * 协议细节：
     *   VESC 接收 int32 = pwm × 100000（放大10万倍存为整数）
     *   → data[0..3] = big-endian int32(pwm × 100000)
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_DUTY(0)
     */
    void setPwm(double pwm);

    /**
     * @brief 设置目标位置（位置控制模式）
     * @param pos  目标位置（整数，单位由 VESC 内部配置决定，通常为角度×1e6）
     *
     * 协议细节：
     *   → data[0..3] = big-endian int32(pos)，直接发送，不做缩放
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_POS(4)
     */
    void setPos(int32_t pos);

    /**
     * @brief 设置制动电流（再生制动 / 电阻制动）
     * @param current_mA  制动电流，单位 mA，应为正值
     *
     * 协议细节：
     *   VESC 期望 μA，协议 int32 = current_mA × 1000
     *   → data[0..3] = big-endian int32(current_mA × 1000)
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_CURRENT_BRAKE(2)
     *
     * 注意：与 setCurrent(负值) 的区别在于：
     *   setCurrent 是四象限电流控制（可加速也可制动）
     *   setBrakeCurrent 是专用制动指令（只制动，不驱动）
     */
    void setBrakeCurrent(int32_t current_mA);

    /**
     * @brief 设置手刹电流（绝对值制动，适合停车保持）
     * @param current_mA  手刹电流，单位 mA，正值
     *
     * 协议细节：
     *   int32 = current_mA × 1000
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_CURRENT_HANDBRAKE(12)
     *
     * 手刹 vs 制动电流：
     *   手刹以位置保持为目标，制动以减速为目标
     */
    void setHandbrakeCurrent(int32_t current_mA);

    /**
     * @brief 设置相对手刹电流（比例制动）
     * @param relative  相对电流比例，范围 0.0 ~ 1.0
     *                  0.0 = 无制动，1.0 = 最大制动电流
     *
     * 协议细节：
     *   int32 = relative × 100000
     *   CAN ID: bits[15:8]=CAN_PACKET_SET_CURRENT_HANDBRAKE_REL(13)
     */
    void setHandbrakeCurrentRel(float relative);

    /* ========================================================
     *  接收处理（RX 方向：VESC → MCU）
     * ======================================================== */

    /**
     * @brief CAN 接收帧处理函数（在 bsp_can 分发层调用）
     * @param rxFrame  指向 BSP_CAN::FDCAN_RxFIFO0Frame 的指针
     *                 由 bsp_can 层在 HAL_FDCAN_RxFifo0Callback 中填充后传入
     *
     * 内部从 rxFrame->Header.Identifier 读取：
     *   - Identifier[7:0]  → node_id（与本对象 nodeId_ 比较，不匹配则 return）
     *   - Identifier[15:8] → cmd（状态类型，switch 分发）
     * 数据从 rxFrame->Data[8] 解析，写入 rxData_。
     *
     * 支持解析的状态类型：
     *   STATUS   → eRPM、机械转速、相电流、占空比
     *   STATUS_4 → PID位置、累计角度（带圈数跟踪）
     *   其余状态帧（STATUS_2/3/5）暂时预留，不解析
     *
     * 使用方式（在 bsp_can.cpp 分发函数里）：
     *   for (auto& m : VescMotors) m.canRxHandler(&BSP_CAN::FDCAN_RxFIFO0Frame);
     */
    void canRxHandler(const FDCAN_RxFrame_TypeDef* rxFrame);

    /* ========================================================
     *  数据访问接口（读取反馈）
     * ======================================================== */

    /**
     * @brief 获取接收数据的 const 引用（只读）
     * @return VescRxData 结构体引用，包含转速、电流、位置等信息
     */
    const VescRxData& getRxData() const { return rxData_; }

    /**
     * @brief 获取节点 ID
     */
    uint16_t getNodeId() const { return nodeId_; }

    /**
     * @brief 获取绑定的 FDCAN 句柄指针
     */
    FDCAN_HandleTypeDef* getFdcan() const { return hfdcan_; }

private:
    /* --------------------------------------------------------
     *  私有成员变量
     * -------------------------------------------------------- */

    FDCAN_HandleTypeDef* hfdcan_;  ///< 绑定的 FDCAN 外设句柄
    uint16_t             nodeId_;  ///< 本电调的 CAN 节点 ID（0~255）
    VescRxData           rxData_;  ///< 最新接收并解析的状态数据

    /**
     * @brief 极对数（pole pairs）
     * 24N28P 电机：28极 / 2 = 14 极对数
     * 此值用于 eRPM ↔ 机械RPM 换算，建议以构造参数或常量传入
     * 当前按原代码硬编码为 14
     */
    static constexpr int POLE_PAIRS = 14;

    /* --------------------------------------------------------
     *  私有辅助函数：构建并发送 FDCAN 扩展数据帧
     *
     *  @param cmd    命令类型（CanPacketID 枚举）
     *  @param data   4字节数据负载（big-endian int32，其余字节填0）
     *
     *  FDCAN 帧参数说明：
     *    IdType      = FDCAN_EXTENDED_ID   → 29位扩展帧，VESC 协议要求
     *    TxFrameType = FDCAN_DATA_FRAME    → 数据帧（非远程帧）
     *    DataLength  = FDCAN_DLC_BYTES_8   → DLC=8，固定8字节负载
     *    Identifier  = nodeId_ | (cmd<<8)  → VESC 协议 ID 格式
     *    ErrorStateIndicator = FDCAN_ESI_ACTIVE  (默认，无错误)
     *    BitRateSwitch       = FDCAN_BRS_OFF     (经典CAN，不切换比特率)
     *    FDFormat            = FDCAN_CLASSIC_CAN (不启用 FD 模式)
     *    TxEventFifoControl  = FDCAN_NO_TX_EVENTS(不记录发送事件)
     *    MessageMarker       = 0
     * -------------------------------------------------------- */
    void sendFrame(CanPacketID cmd, const uint8_t data[8]);

    /**
     * @brief 将 int32_t 按大端序拆分到 data[0..3]
     * @param val   要拆分的32位整数（补码）
     * @param data  输出字节数组（至少4字节），data[0]=高字节
     *
     * VESC 协议统一使用大端序（Big-Endian / MSB first）：
     *   data[0] = (val >> 24) & 0xFF  // 最高字节
     *   data[1] = (val >> 16) & 0xFF
     *   data[2] = (val >>  8) & 0xFF
     *   data[3] = (val      ) & 0xFF  // 最低字节
     */
    static void packInt32BigEndian(int32_t val, uint8_t* data);
};

/* ============================================================
 *  全局电机数组（与原代码 Vesc_Motor_U8[4] 对应）
 *  下标 0~3 对应四个电机，使用前需各自调用 init()
 * ============================================================ */
extern VescMotor VescMotors[4];
