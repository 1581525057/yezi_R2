#include "AS5047.h"

/* ======================== GPIO Helpers =================================== */
#define AS5047P_CS_LOW()  HAL_GPIO_WritePin(AS5047P_CS_PORT, AS5047P_CS_PIN, GPIO_PIN_RESET) // 片选拉低：开始一次新的 16 位 SPI 事务。
#define AS5047P_CS_HIGH() HAL_GPIO_WritePin(AS5047P_CS_PORT, AS5047P_CS_PIN, GPIO_PIN_SET)   // 片选拉高：结束本次 SPI 事务，让芯片锁存命令或移出数据。

AS5047P as5047;

/*
 * 模块内部工作流程：
 * 1. 主机发送 16 位命令帧给 AS5047P。
 * 2. 读取寄存器时需要“两拍”操作：
 *    第一拍发送 READ + 目标地址；
 *    第二拍发送 NOP，把上一拍请求的数据从 MISO 推出来。
 * 3. 返回帧 bit14 为 EF(error flag)，只表示“上一帧命令存在错误提示”。
 * 4. 若要查看具体错误原因，需要继续读取 ERRFL。
 * 5. ERRFL 为读后清零寄存器；DIAAGC 则可用于持续观察磁场和内部状态。
 */

/* ======================== SPI Low Level ================================== */

uint16_t AS5047P::AS5047P_SPI_Transfer16(uint16_t txData)
{
    uint8_t txBuf[2];
    uint8_t rxBuf[2];
    uint16_t rxData;

    // AS5047P 按高字节在前的顺序进行 16 位 SPI 传输。
    txBuf[0] = (uint8_t)(txData >> 8);
    txBuf[1] = (uint8_t)(txData & 0xFF);

    // 一次完整 16 位事务期间，片选必须保持有效。
    AS5047P_CS_LOW();
    HAL_SPI_TransmitReceive(spi, txBuf, rxBuf, 2, 100);
    AS5047P_CS_HIGH();

    // 接收到的数据同样按高字节在前方式拼回 16 位原始帧。
    rxData = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
    return rxData;
}

/*
 * AS5047P 命令帧要求偶校验。
 * 对于主机发出的 16 位命令：
 *   bit15 = parity
 *   bit14 = R/W
 *   bit13:0 = address
 * bit15 需要根据低 15 位计算，使“整个 16 位命令中的 1 的个数”为偶数。
 */
uint8_t AS5047P::AS5047P_EvenParity15(uint16_t value)
{
    // 只对低 15 位做统计，因为 bit15 本身就是待生成的奇偶校验位。
    value &= 0x7FFF;

    uint8_t cnt = 0;
    for (int i = 0; i < 15; i++) {
        if (value & (1U << i)) {
            cnt++;
        }
    }

    // 若低 15 位中“1”的个数为奇数，则 bit15 必须置 1，才能让总数变成偶数。
    return (cnt & 0x01U) ? 1U : 0U;
}

/*
 * 组装一帧 AS5047P 命令。
 * 命令格式：
 *   bit15 = parity
 *   bit14 = R/W，1 表示读，0 表示写
 *   bit13:0 = register address
 */
uint16_t AS5047P::AS5047P_MakeCommand(uint16_t addr, uint8_t isRead)
{
    uint16_t cmd = 0;

    // 先填入 14 位寄存器地址。
    cmd |= (addr & 0x3FFFU);

    // 读命令时将 bit14 置 1；当前模块只实现读流程。
    if (isRead) {
        cmd |= (1U << 14);
    }

    // 根据低 15 位结果生成偶校验位，并放到 bit15。
    if (AS5047P_EvenParity15(cmd)) {
        cmd |= (1U << 15);
    }

    return cmd;
}

/*
 * 读取任意寄存器的原始返回帧。
 * 重点：AS5047P 读寄存器不是“一拍就拿到结果”，而是典型的流水线式两拍读取：
 *   1. 第一拍发送 READ addr；
 *   2. 第二拍发送 NOP；
 *   3. 第二拍返回值中才携带第一拍请求的寄存器内容。
 */
uint16_t AS5047P::AS5047P_ReadRaw(uint16_t addr)
{
    uint16_t cmd;
    uint16_t rx;

    // 第一拍：告诉芯片“我要读哪个寄存器”。
    cmd = AS5047P_MakeCommand(addr, 1);
    AS5047P_SPI_Transfer16(cmd);

    // 第二拍：发送 NOP，把上一拍请求的数据从 MISO 移出来。
    cmd = AS5047P_MakeCommand(AS5047P_REG_NOP, 1);
    rx  = AS5047P_SPI_Transfer16(cmd);

    return rx;
}

uint8_t AS5047P::AS5047P_CheckErrorFlag(uint16_t rawFrame)
{
    // 返回帧 bit14 为 EF(error flag)。
    // 它只说明“上一帧命令被芯片判定存在错误提示”，并不直接说明具体错误类型。
    return (rawFrame & 0x4000U) ? 1U : 0U;
}

uint16_t AS5047P::  AS5047P_GetData14(uint16_t rawFrame)
{
    // 返回帧低 14 位才是真正的数据载荷。
    return (rawFrame & 0x3FFFU);
}

void AS5047P::AS5047P_UpdateErrflCache(uint16_t errflData)
{
    // ERRFL 只有低 3 位有效，其余位保留；这里保留原始 14 位数据，便于调试。
    data.errfl_raw = errflData;

    data.errfl_parerr  = (errflData >> AS5047P_ERRFL_PARERR_BIT)  & 0x01U;
    data.errfl_invcomm = (errflData >> AS5047P_ERRFL_INVCOMM_BIT) & 0x01U;
    data.errfl_frerr   = (errflData >> AS5047P_ERRFL_FRERR_BIT)   & 0x01U;
}

void AS5047P::AS5047P_UpdateDiaagcCache(uint16_t diaagcData)
{
    // 保存 DIAAGC 原始值，便于后续对照手册定位问题。
    data.diaagc_raw = diaagcData;

    data.diaagc_magl = (diaagcData >> AS5047P_DIAAGC_MAGL_BIT) & 0x01U;
    data.diaagc_magh = (diaagcData >> AS5047P_DIAAGC_MAGH_BIT) & 0x01U;
    data.diaagc_cof  = (diaagcData >> AS5047P_DIAAGC_COF_BIT)  & 0x01U;
    data.diaagc_lf   = (diaagcData >> AS5047P_DIAAGC_LF_BIT)   & 0x01U;
    data.diaagc_agc  = (uint8_t)(diaagcData & AS5047P_DIAAGC_AGC_MASK);
}

uint16_t AS5047P::AS5047P_ReadAngleRaw(void)
{
    // 读取 ANGLECOM 返回帧后，仅提取低 14 位角度原始码值。
    uint16_t rawFrame = AS5047P_ReadRaw(AS5047P_REG_ANGLECOM);
    return AS5047P_GetData14(rawFrame);
}

float AS5047P::AS5047P_ReadAngleDegree(void)
{
    // 14 位满量程一共 16384 个刻度，对应 360 度。
    uint16_t raw = AS5047P_ReadAngleRaw();
    return (360.0f * raw) / 16384.0f;
}

/* ======================== Public Interface =============================== */

void AS5047P::init(SPI_HandleTypeDef *hspi)
{
    // 保存外部传入的 SPI 句柄，后续所有寄存器访问都依赖它。
    spi = hspi;

    // 初始化角度缓存。
    data.angle = 0.0f;
    data.angle_raw = 0U;
    data.frame_error_flag = 0U;

    // 初始化 ERRFL 缓存。
    data.errfl_raw = 0U;
    data.errfl_parerr = 0U;
    data.errfl_invcomm = 0U;
    data.errfl_frerr = 0U;

    // 初始化 DIAAGC 缓存。
    data.diaagc_raw = 0U;
    data.diaagc_magl = 0U;
    data.diaagc_magh = 0U;
    data.diaagc_cof = 0U;
    data.diaagc_lf = 0U;
    data.diaagc_agc = 0U;
}

uint16_t AS5047P::readErrorRegister(void)
{
    // ERRFL 为读后清零寄存器。
    // 因此这里既返回本次读取值，也把它写入本地缓存，方便后续调试查看。
    uint16_t rawFrame = AS5047P_ReadRaw(AS5047P_REG_ERRFL);
    uint16_t errflData = AS5047P_GetData14(rawFrame);

    AS5047P_UpdateErrflCache(errflData);
    return errflData;
}

uint16_t AS5047P::readDiagAgcRegister(void)
{
    // DIAAGC 不是清零型寄存器，可以按需频繁读取。
    uint16_t rawFrame = AS5047P_ReadRaw(AS5047P_REG_DIAAGC);
    uint16_t diaagcData = AS5047P_GetData14(rawFrame);

    AS5047P_UpdateDiaagcCache(diaagcData);
    return diaagcData;
}

void AS5047P::updata(void)
{
    // 第 1 步：读取补偿后角度寄存器 ANGLECOM 的完整返回帧。
    uint16_t angleFrame = AS5047P_ReadRaw(AS5047P_REG_ANGLECOM);

    // 第 2 步：提取 14 位角度原始码值，并换算成角度值。
    data.angle_raw = AS5047P_GetData14(angleFrame);
    data.angle = (360.0f * data.angle_raw) / 16384.0f;

    // 第 3 步：记录本次角度读取返回帧中的 EF 错误标志。
    data.frame_error_flag = AS5047P_CheckErrorFlag(angleFrame);
 
    // 第 4 步：若 EF=1，说明芯片提示上一帧命令存在错误，需要继续读取 ERRFL 看具体原因。
    // 读取 ERRFL 会清除芯片内部错误锁存，但本地缓存会保留最后一次读到的结果。
    if (data.frame_error_flag) {
        readErrorRegister();
    }

    // 第 5 步：每次更新都读取一次 DIAAGC，持续观察磁场强弱、CORDIC 状态和 AGC 值。
    readDiagAgcRegister();
}
