/**
 * @file    VescMotor.cpp
 * @brief   VESC电调 CAN通信驱动实现 —— C++ 版本
 * @author  重构自 vesc_can.c
 * @date    2025
 *
 * ============================================================
 *  【移植检查清单】
 * ============================================================
 *  □ 1. CubeMX 中 FDCANx 已配置并生成初始化代码
 *  □ 2. 波特率设置为 500 kbps（VESC 默认 CAN 波特率）
 *         CubeMX 参数参考（以 80MHz 时钟为例）：
 *           Prescaler       = 8
 *           Nominal Time Seg1 = 13
 *           Nominal Time Seg2 = 2
 *           → 实际波特率 = 80MHz / (8 × (1+13+2)) = 625kHz（需调整）
 *         实际请用 VESC Tool 确认波特率
 *  □ 3. 已调用 HAL_FDCAN_ConfigFilter / HAL_FDCAN_ConfigGlobalFilter
 *  □ 4. 已调用 HAL_FDCAN_Start(&hfdcanX)
 *  □ 5. 已调用 HAL_FDCAN_ActivateNotification 启用 FIFO0 中断
 *  □ 6. 在 HAL_FDCAN_RxFifo0Callback 中调用 motor.canRxHandler()
 *  □ 7. bsp_can.h 中 FDCAN_RxFrame_TypeDef 定义了 .Header.Identifier
 *
 * ============================================================
 *  【接收中断接入示例（stm32h7xx_it.c 或 main.c）】
 * ============================================================
 *
 *  // bsp_can 层全局接收缓存（在 bsp_can.c 中定义）
 *  extern FDCAN_RxFrame_TypeDef FIFO0_FDCAN_RxDATA;
 *
 *  // HAL FDCAN 接收 FIFO0 非空回调（HAL 库自动调用）
 *  void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
 *                                  uint32_t RxFifo0ITs)
 *  {
 *      // 1. 获取原始帧（HAL 层），填入全局缓存
 *      HAL_FDCAN_GetRxMessage(hfdcan,
 *                             FDCAN_RX_FIFO0,
 *                             &FIFO0_FDCAN_RxDATA.Header,
 *                             FIFO0_FDCAN_RxDATA.Data);
 *
 *      // 2. 分发给各电机对象处理
 *      //    motor.canRxHandler 内部会按 node_id 过滤，不匹配则忽略
 *      for (auto& motor : VescMotors) {
 *          motor.canRxHandler(hfdcan, FIFO0_FDCAN_RxDATA.Data);
 *      }
 *  }
 *
 * ============================================================
 */

#include "VescMotor.hpp"
#include "fdcan.h"   // CubeMX 生成的 FDCAN 初始化声明
#include "gpio.h"    // GPIO 初始化（可选，部分板子需要）
#include <cstring>   // memset

/* ============================================================
 *  全局电机数组定义（4 个电机实例）
 *  使用前需各自调用 VescMotors[i].init(&hfdcanX, id)
 * ============================================================ */
VescMotor VescMotors[4];

/* ============================================================
 *  构造函数
 *  所有指针清零，数值清零，C++ 会自动调用 VescRxData 的默认构造
 * ============================================================ */
VescMotor::VescMotor()
    : hfdcan_(nullptr)
    , nodeId_(0)
    , rxData_()  // 调用 VescRxData 默认构造，全部清零
{
}

/* ============================================================
 *  init() —— 绑定外设与节点ID
 *
 *  对应原 VESC_Motor_Struct_Init()，但不再需要赋值函数指针：
 *    原来：vesc_motor->SetCurrent = VESC_Set_Current_Struct;
 *    现在：直接通过 motor.setCurrent() 调用，无需指针
 * ============================================================ */
void VescMotor::init(FDCAN_HandleTypeDef* hfdcan, uint16_t nodeId)
{
    hfdcan_ = hfdcan;
    nodeId_ = nodeId;
    // rxData_ 已在构造函数清零，不需要再次清零
}

/* ============================================================
 *  私有辅助：大端序打包 int32 → 4字节数组
 *
 *  VESC CAN 协议数据格式：big-endian（高字节在低地址）
 *  这与 x86/ARM 的 little-endian 内存布局相反，必须手动拆字节
 *
 *  例：val = 0x12345678
 *    data[0] = 0x12  (最高字节，bits31..24)
 *    data[1] = 0x34  (bits23..16)
 *    data[2] = 0x56  (bits15..8)
 *    data[3] = 0x78  (最低字节，bits7..0)
 * ============================================================ */
void VescMotor::packInt32BigEndian(int32_t val, uint8_t* data)
{
    // 右移后与 0xFF 掩码，安全提取每个字节
    // 注意：对有符号数右移在 C++ 中是实现定义行为，但 VESC 原码也这样用，
    //       在 ARM Cortex-M（算术右移）上是正确的。
    data[0] = static_cast<uint8_t>((val >> 24) & 0xFF);  // 最高字节
    data[1] = static_cast<uint8_t>((val >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((val >>  8) & 0xFF);
    data[3] = static_cast<uint8_t>((val      ) & 0xFF);  // 最低字节
}

/* ============================================================
 *  私有辅助：构建并通过 BSP_CAN 统一接口发送 FDCAN 扩展数据帧
 *
 *  【为什么改用 BSP_CAN::AddMessageToTxFifoQ 而非直接调 HAL】
 *  原版直接调 HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, ...)，
 *  绕过了 BSP_CAN 层的统一发送管理。改用 BSP_CAN 接口后：
 *    1. 发送逻辑集中在 bsp_can.cpp，维护更方便
 *    2. 发送帧对象（FDCAN_TxFrame_TypeDef）包含 hcan 指针，
 *       BSP_CAN::AddMessageToTxFifoQ 内部直接取用，无需在外部传句柄
 *    3. 与 DJI/达妙等其他电机驱动的发送方式保持一致
 *
 *  【如何选择 TxFrame 对象】
 *  BSP_CAN 提供三个预置发送帧对象，分别对应三路 FDCAN：
 *    BSP_CAN::FDCAN1_TxFrame  → hfdcan1
 *    BSP_CAN::FDCAN2_TxFrame  → hfdcan2
 *    BSP_CAN::FDCAN3_TxFrame  → hfdcan3
 *  本函数根据 hfdcan_ 指针选择对应的 TxFrame，
 *  填充 Header（ID、类型、长度）和 Data 后，
 *  调用 BSP_CAN::AddMessageToTxFifoQ 提交到 TX FIFO。
 *
 *  【FDCAN_TxHeaderTypeDef 字段详解】
 *  ┌─────────────────────────────────────────────────────────┐
 *  │ Identifier         │ 29位扩展ID：低8位=nodeId_，高8位=cmd│
 *  │ IdType             │ FDCAN_EXTENDED_ID（VESC 用29位扩展）│
 *  │ TxFrameType        │ FDCAN_DATA_FRAME（数据帧，非远程帧）│
 *  │ DataLength         │ FDCAN_DLC_BYTES_8（固定8字节）      │
 *  │ ErrorStateIndicator│ FDCAN_ESI_ACTIVE（主动错误状态）    │
 *  │ BitRateSwitch      │ FDCAN_BRS_OFF（经典CAN不切比特率）  │
 *  │ FDFormat           │ FDCAN_CLASSIC_CAN（不用FD扩展格式）│
 *  │ TxEventFifoControl │ FDCAN_NO_TX_EVENTS（不记录发送事件）│
 *  │ MessageMarker      │ 0（用户标签，此处不用）             │
 *  └─────────────────────────────────────────────────────────┘
 * ============================================================ */
void VescMotor::sendFrame(CanPacketID cmd, const uint8_t data[8])
{
    /* --- 根据绑定的 hfdcan_ 指针选择对应的 BSP_CAN TxFrame 对象 ---
     *  BSP_CAN 为每路 FDCAN 预置了一个静态 TxFrame，
     *  内部已含 .hcan 指针，AddMessageToTxFifoQ 直接使用它。
     *  若 hfdcan_ 不匹配任何一路（配置错误），直接返回，不发送。
     */
    FDCAN_TxFrame_TypeDef* txFrame = nullptr;

    if      (hfdcan_ == &hfdcan1) txFrame = &BSP_CAN::FDCAN1_TxFrame;
    else if (hfdcan_ == &hfdcan2) txFrame = &BSP_CAN::FDCAN2_TxFrame;
    else if (hfdcan_ == &hfdcan3) txFrame = &BSP_CAN::FDCAN3_TxFrame;
    else                          return;   // 未知外设，防御性返回

    /* --- 填充帧头 ---
     *  VESC 协议 29 位扩展 ID 格式：
     *    bits[ 7: 0] = 节点 ID（0~255）
     *    bits[15: 8] = 命令类型（CanPacketID）
     *    bits[28:16] = 0（VESC 协议未使用高位）
     */
    txFrame->Header.IdType      = FDCAN_EXTENDED_ID;   // 29位扩展帧
    txFrame->Header.TxFrameType = FDCAN_DATA_FRAME;    // 数据帧
    txFrame->Header.DataLength  = FDCAN_DLC_BYTES_8;   // 固定8字节
    txFrame->Header.Identifier  =
        (static_cast<uint32_t>(nodeId_) & 0xFF) |      // 低8位 = 节点ID
        (static_cast<uint32_t>(cmd) << 8);             // 高8位 = 命令类型

    // FD/错误状态字段（经典CAN模式下保持默认，HAL 要求必须填写）
    txFrame->Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txFrame->Header.BitRateSwitch       = FDCAN_BRS_OFF;
    txFrame->Header.FDFormat            = FDCAN_CLASSIC_CAN;
    txFrame->Header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txFrame->Header.MessageMarker       = 0;

    /* --- 复制数据负载（8字节）--- */
    // 使用 memcpy 而非手动循环，编译器可优化为单条 LDM/STM 指令
    memcpy(txFrame->Data, data, 8);

    /* --- 通过 BSP_CAN 统一接口提交到 TX FIFO Queue ---
     *  BSP_CAN::AddMessageToTxFifoQ 内部调用：
     *    HAL_FDCAN_AddMessageToTxFifoQ(txFrame->hcan, &txFrame->Header, txFrame->Data)
     *  TX FIFO 满时 HAL 返回 HAL_ERROR，此处忽略（量产代码可加重试逻辑）
     */
    BSP_CAN::AddMessageToTxFifoQ(txFrame);
}

/* ============================================================
 *  setCurrent() —— 设置目标电流（力矩控制）
 *
 *  【协议编码】
 *  VESC 期望：int32（μA精度）= current_mA × 1000
 *  范围：受 VESC Tool 中配置的 Motor Max Current 限制，超出会被 clamp
 *
 *  数据帧结构（8字节，只用前4字节）：
 *  ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 *  │Byte0 │Byte1 │Byte2 │Byte3 │Byte4 │Byte5 │Byte6 │Byte7 │
 *  │ current(μA) big-endian int32  │      0       │      0       │
 *  └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 * ============================================================ */
void VescMotor::setCurrent(int32_t current_mA)
{
    uint8_t data[8] = {0};  // 全部清零，未使用字节保持0

    // mA → μA（× 1000）：VESC 协议要求单位为 μA 精度的 int32
    // 注意溢出：current_mA 最大约 ±2,147,483 mA（int32_t范围），
    //           实际电机电流远小于此，无溢出风险
    const int32_t current_uA = current_mA * 1000;

    // 大端序打包到 data[0..3]
    packInt32BigEndian(current_uA, data);

    // 发送命令帧
    sendFrame(CanPacketID::SET_CURRENT, data);
}

/* ============================================================
 *  setRpm() —— 设置目标转速（速度控制）
 *
 *  【极对数换算】
 *  VESC 内部始终使用电气转速 eRPM：
 *    eRPM = 机械RPM × 极对数
 *  本驱动在此函数内完成换算，调用者传入机械转速即可。
 *
 *  【协议编码】
 *  数据帧：big-endian int32(eRPM)，单位 eRPM/min
 *
 *  例：机械转速 300 RPM，极对数 14
 *    → 发送 eRPM = 300 × 14 = 4200
 * ============================================================ */
void VescMotor::setRpm(int32_t rpm)
{
    uint8_t data[8] = {0};

    // 机械转速 → 电气转速（eRPM）
    // 注意：RPM 较大时（>±153,391 RPM）会溢出 int32_t，实际电机不会达到此值
    const int32_t erpm = rpm * POLE_PAIRS;

    packInt32BigEndian(erpm, data);

    sendFrame(CanPacketID::SET_RPM, data);
}

/* ============================================================
 *  setPwm() —— 设置占空比（开环电压控制）
 *
 *  【协议编码】
 *  VESC 期望：int32 = pwm × 100000（10万倍放大为整数）
 *  范围：-100000（-100%）~ +100000（+100%）
 *  double → int32 转换时注意截断 vs 四舍五入（原码用截断）
 *
 *  数据帧：big-endian int32(pwm×100000)
 * ============================================================ */
void VescMotor::setPwm(double pwm)
{
    uint8_t data[8] = {0};

    // double → int32，×100000 放大
    // static_cast<int32_t> 为截断（向零取整），与原代码一致
    const int32_t pwm_scaled = static_cast<int32_t>(pwm * 100000.0);

    packInt32BigEndian(pwm_scaled, data);

    sendFrame(CanPacketID::SET_DUTY, data);
}

/* ============================================================
 *  setPos() —— 设置目标位置（位置控制）
 *
 *  【协议编码】
 *  原码直接发送 int32，VESC 内部单位通常为 度×1000000，
 *  具体取决于 VESC Tool 的 "Encoder" 配置。
 *  本函数直接传入原始 int32 值，不做任何缩放。
 * ============================================================ */
void VescMotor::setPos(int32_t pos)
{
    uint8_t data[8] = {0};

    packInt32BigEndian(pos, data);

    sendFrame(CanPacketID::SET_POS, data);
}

/* ============================================================
 *  setBrakeCurrent() —— 设置制动电流
 *
 *  【与 setCurrent 的区别】
 *  setCurrent(负值) → 反转驱动电流（可以加速也可以制动）
 *  setBrakeCurrent  → 专用制动指令，不管转向，只做制动
 *
 *  【协议编码】与 setCurrent 相同：int32 = current_mA × 1000
 * ============================================================ */
void VescMotor::setBrakeCurrent(int32_t current_mA)
{
    uint8_t data[8] = {0};

    const int32_t current_uA = current_mA * 1000;

    packInt32BigEndian(current_uA, data);

    sendFrame(CanPacketID::SET_CURRENT_BRAKE, data);
}

/* ============================================================
 *  setHandbrakeCurrent() —— 设置手刹电流（绝对值）
 *
 *  手刹电流使 VESC 进入"位置保持"状态，常用于停车锁定。
 *  调用 setCurrent(0) 后电机自由转动，调用 setHandbrakeCurrent
 *  后电机被锁定在当前位置。
 *
 *  【协议编码】int32 = current_mA × 1000
 * ============================================================ */
void VescMotor::setHandbrakeCurrent(int32_t current_mA)
{
    uint8_t data[8] = {0};

    const int32_t send_current = current_mA * 1000;

    packInt32BigEndian(send_current, data);

    sendFrame(CanPacketID::SET_CURRENT_HANDBRAKE, data);
}

/* ============================================================
 *  setHandbrakeCurrentRel() —— 设置相对手刹电流（比例）
 *
 *  @param relative  0.0 ~ 1.0，对应 0% ~ 100% 最大电流
 *
 *  【协议编码】int32 = relative × 100000
 *  与 setPwm 编码方式相同，都是×10万放大为整数
 * ============================================================ */
void VescMotor::setHandbrakeCurrentRel(float relative)
{
    uint8_t data[8] = {0};

    // float → int32，×100000 放大（截断）
    const int32_t send_value = static_cast<int32_t>(relative * 100000.0f);

    packInt32BigEndian(send_value, data);

    sendFrame(CanPacketID::SET_CURRENT_HANDBRAKE_REL, data);
}

/* ============================================================
 *  canRxHandler() —— CAN 接收帧解析（由 bsp_can 分发层调用）
 *
 *  【调用方式（bsp_can.cpp 中）】
 *  void BSP_CAN::FDCAN1_RxFifo0RxHandler(uint32_t*, uint8_t[8])
 *  {
 *      for (auto& m : VescMotors)
 *          m.canRxHandler(&BSP_CAN::FDCAN_RxFIFO0Frame);
 *  }
 *
 *  【参数说明】
 *  @param rxFrame  指向已由 HAL_FDCAN_GetRxMessage 填充的帧结构体
 *                  rxFrame->Header.Identifier  = 29位扩展帧ID
 *                  rxFrame->Data[8]            = 8字节数据负载
 *
 *  【VESC 接收帧 ID 解析】
 *  扩展帧 29位 Identifier：
 *    bits[ 7: 0] = VESC 节点 ID（谁发的，用于过滤）
 *    bits[15: 8] = 状态类型（CAN_PACKET_STATUS / STATUS_2 / ...）
 *
 *  【数据大端序解析】
 *  所有多字节字段均为 big-endian，需手动重新拼接：
 *    int16 = (int16_t)(data[n] << 8) | data[n+1]
 *    int32 = (data[0]<<24)|(data[1]<<16)|(data[2]<<8)|data[3]
 *
 * ============================================================ */
void VescMotor::canRxHandler(const FDCAN_RxFrame_TypeDef* rxFrame)
{
    // 从传入帧读取 29 位扩展 ID（由 bsp_can 的 HAL_FDCAN_GetRxMessage 已填充）
    const uint32_t rawId = rxFrame->Header.Identifier;

    // 解析发送方节点 ID（低8位）
    // 注意：nodeId_ 是 uint16_t，rxNodeId 用 uint8_t 比较，范围 0~255
    const uint8_t rxNodeId = static_cast<uint8_t>(rawId & 0xFF);

    // 解析命令/状态类型（第9~16位）
    const uint8_t cmd = static_cast<uint8_t>((rawId >> 8) & 0xFF);

    // 只处理与本电机 nodeId_ 匹配的帧，过滤掉总线上其他 VESC 的广播
    if (rxNodeId != static_cast<uint8_t>(nodeId_))
    {
        return;  // ID 不匹配，直接返回，不解析
    }

    // 取数据指针（避免后续重复写 rxFrame->Data）
    const uint8_t* data = rxFrame->Data;

    // 根据命令类型分发解析逻辑
    switch (static_cast<CanPacketID>(cmd))
    {
        /* ----------------------------------------------------
         *  状态帧1（STATUS）：包含 eRPM、相电流、占空比
         *
         *  数据布局（8字节 big-endian）：
         *  ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
         *  │Byte0 │Byte1 │Byte2 │Byte3 │Byte4 │Byte5 │Byte6 │Byte7 │
         *  │符号标志(0xFF=负)  │eRPM低16位绝对值│电流×10(A) │占空比×1000  │
         *  └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
         *
         *  eRPM 原码解析约定（沿用原始 C 代码逻辑）：
         *    若 data[0]==0xFF && data[1]==0xFF → 负转速
         *      对 Byte2/Byte3 按位取反后作为绝对值，加负号
         *    否则 → 正转速，直接用 Byte2/Byte3 拼接
         *  注：此方式只覆盖 ±65535 eRPM（约 ±4681 RPM@14极对），
         *      若需更高转速请改为标准 int32 big-endian 解析
         * ---------------------------------------------------- */
        case CanPacketID::STATUS:
        {
            // eRPM 解析：判断高两字节是否为 0xFFFF（负转速标志）
            if (data[0] == 0xFF && data[1] == 0xFF)
            {
                // 负转速：对 Byte2/Byte3 按位取反得绝对值，取负
                const float erpm_abs = static_cast<float>(
                    (static_cast<uint16_t>(~data[2] & 0xFF) << 8) |
                    (static_cast<uint16_t>(~data[3] & 0xFF))
                );
                rxData_.eRpm = -erpm_abs;
            }
            else
            {
                // 正转速：直接拼接 Byte2/Byte3
                rxData_.eRpm = static_cast<float>(
                    (static_cast<uint16_t>(data[2]) << 8) |
                    (static_cast<uint16_t>(data[3]))
                );
            }

            // 机械转速 = eRPM / 极对数
            rxData_.rpm = rxData_.eRpm / static_cast<float>(POLE_PAIRS);

            // 相电流（A）= int16(Byte4, Byte5) / 10.0
            // VESC 发送：电流×10（0.1A精度），16位有符号整数，big-endian
            rxData_.totalCurrent = static_cast<float>(
                static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) |
                                      static_cast<uint16_t>(data[5]))
            ) / 10.0f;

            // 占空比 = int16(Byte6, Byte7) / 1000.0
            // VESC 发送：占空比×1000（0.001精度），16位有符号整数，big-endian
            rxData_.duty = static_cast<float>(
                static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) |
                                      static_cast<uint16_t>(data[7]))
            ) / 1000.0f;

            rxData_.dutyCycle = rxData_.duty;  // 保持兼容
            break;
        }

        /* ----------------------------------------------------
         *  状态帧2（STATUS_2）：安时消耗（暂未使用）
         *
         *  数据布局：
         *  Byte0-3：安时消耗（Ah×1e4，int32 big-endian）
         *  Byte4-7：安时回充（Ah×1e4，int32 big-endian）
         *
         *  解析示例（当需要启用时）：
         *    float amp_hours = (int32解析 data[0..3]) / 1e4f;
         * ---------------------------------------------------- */
        case CanPacketID::STATUS_2:
        {
            // 暂未使用，保留扩展接口
            break;
        }

        /* ----------------------------------------------------
         *  状态帧3（STATUS_3）：瓦时消耗（暂未使用）
         * ---------------------------------------------------- */
        case CanPacketID::STATUS_3:
        {
            break;
        }

        /* ----------------------------------------------------
         *  状态帧4（STATUS_4）：PID位置 + 温度 + 输入电流
         *
         *  数据布局（8字节 big-endian）：
         *  ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
         *  │Byte0 │Byte1 │Byte2 │Byte3 │Byte4 │Byte5 │Byte6 │Byte7 │
         *  │MOSFET温度×10  │电机温度×10   │输入电流×10  │PID角度×50  │
         *  └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
         *
         *  当前只解析 PID 角度（Byte6, Byte7）：
         *    pidPositionNow = int16(Byte6, Byte7) / 50.0f → 0~360°
         *
         *  【累计角度跟踪算法】
         *  VESC 发送的 PID 角度范围 0~360°，跨零时发生跳变。
         *    delta < -180° → 正转过零（359°→0°），turnCount++
         *    delta > +180° → 反转过零（0°→359°），turnCount--
         *    totalPosition = turnCount × 360 + pidPositionNow
         * ---------------------------------------------------- */
        case CanPacketID::STATUS_4:
        {
            // 保存上次角度，用于圈数跟踪
            rxData_.pidPositionLast = rxData_.pidPositionNow;

            // PID 当前角度：Byte6/Byte7 组成 int16，除以50得到度数（精度0.02°）
            rxData_.pidPositionNow = static_cast<float>(
                static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) |
                                      static_cast<uint16_t>(data[7]))
            ) / 50.0f;

            // 圈数跟踪：处理 0°/360° 边界跳变
            const float delta = rxData_.pidPositionNow - rxData_.pidPositionLast;

            if (delta < -180.0f)
            {
                rxData_.turnCount++;   // 正转经过 360°→0° 边界
            }
            else if (delta > 180.0f)
            {
                rxData_.turnCount--;   // 反转经过 0°→360° 边界
            }

            // 累计总角度
            rxData_.totalPosition =
                static_cast<float>(rxData_.turnCount) * 360.0f +
                rxData_.pidPositionNow;

            // 以下字段未启用（注释保留，方便按需开启）：
            // float temp_fet   = int16(data[0], data[1]) / 10.0f;
            // float temp_motor = int16(data[2], data[3]) / 10.0f;
            // float current_in = int16(data[4], data[5]) / 10.0f;
            break;
        }

        /* ----------------------------------------------------
         *  状态帧5（STATUS_5）：输入电压 + 里程计数（暂未使用）
         * ---------------------------------------------------- */
        case CanPacketID::STATUS_5:
        {
            break;
        }

        /* 其余命令类型（发送命令的回声等）忽略 */
        default:
            break;
    }
}
