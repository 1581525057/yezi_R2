#include "DT35.h"

/*
 * 全局 DT35 对象。
 * 工程中其他模块通常通过这个单例对象完成初始化、更新和数据读取。
 */
DT35 dt35;

/* ======================== GPIO Helpers =================================== */
#define ADS8688_CS_LOW()    HAL_GPIO_WritePin(ADS8688_CS_PORT,  ADS8688_CS_PIN,  GPIO_PIN_RESET) // 片选拉低：通知 ADS8688 当前 SPI 事务开始，后续时钟和数据对本器件有效。
#define ADS8688_CS_HIGH()   HAL_GPIO_WritePin(ADS8688_CS_PORT,  ADS8688_CS_PIN,  GPIO_PIN_SET)   // 片选拉高：通知 ADS8688 当前 SPI 事务结束，锁存本次命令或配置。
#define ADS8688_RST_LOW()   HAL_GPIO_WritePin(ADS8688_RST_PORT, ADS8688_RST_PIN, GPIO_PIN_RESET) // 硬件复位脚拉低：强制芯片进入复位状态，清空内部运行状态。
#define ADS8688_RST_HIGH()  HAL_GPIO_WritePin(ADS8688_RST_PORT, ADS8688_RST_PIN, GPIO_PIN_SET)   // 释放硬件复位：让芯片重新开始工作，等待后续软件配置。

/*
 * 模块内部数据流说明：
 * 1. 外部先调用 init()，把底层 SPI 句柄交给本模块。
 * 2. init() 内部调用 ADS8688_Init()，完成硬件复位、软件复位、CH0 和量程配置。
 * 3. update() 每调用一次，就通过 SPI 从 ADS8688 读取一次 16 位原始 ADC 数据。
 * 4. 原始 ADC 值按 0 ~ 10.24V 满量程换算为输入电压。
 * 5. 输入电压再按 DT35 的标定关系，从 0V->50mm、10V->10000mm 做线性映射。
 * 6. 结果最终保存到 data 结构体，供外部模块直接读取。
 */

/* ======================== ADS8688 SPI Protocol ============================ */

void DT35::ADS8688_WriteCmd(uint16_t cmd)
{
    // ADS8688 命令传输使用 4 字节 SPI 事务。
    // 前 2 个字节是真正的 16 位命令字，高字节先发，低字节后发。
    // 后 2 个字节填 0x00，主要是为了补齐一次完整事务，同时接收芯片返回的流水线数据。
    uint8_t tx[4] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF), 0x00, 0x00};

    // 接收缓存虽然当前函数不解析，但仍然需要提供给 HAL_SPI_TransmitReceive。
    // 这样可以保证 SPI 时钟完整输出，也便于未来调试时查看返回内容。
    uint8_t rx[4];

    // 拉低 CS，明确告诉 ADS8688：下面这 4 个字节属于同一次命令事务。
    ADS8688_CS_LOW();

    // 全双工收发 4 个字节。
    // 当前最关心的是把命令发出去，因此返回值 rx 未参与后续计算。
    HAL_SPI_TransmitReceive(spi, tx, rx, 4, 100);

    // 拉高 CS，结束本次命令事务，让 ADS8688 执行刚收到的命令。
    ADS8688_CS_HIGH();
}

void DT35::ADS8688_WriteReg(uint8_t addr, uint8_t val)
{
    // 寄存器写操作共发送 2 个字节：
    // 第 1 字节是寄存器地址左移 1 位后，再把最低位置 1，表示“写寄存器”。
    // 第 2 字节是要写入目标寄存器的 8 位数据。
    uint8_t tx[2] = {(uint8_t)((addr << 1) | 0x01), val};

    // 写寄存器前先选中 ADS8688。
    ADS8688_CS_LOW();

    // 发送 2 字节写命令。
    HAL_SPI_Transmit(spi, tx, 2, 100);

    // 发送结束后释放片选，通知芯片本次寄存器写入完成。
    ADS8688_CS_HIGH();
}

uint8_t DT35::ADS8688_ReadReg(uint8_t addr)
{
    // 寄存器读操作共进行 3 字节事务：
    // 第 1 字节是寄存器地址左移 1 位，最低位为 0，表示“读寄存器”。
    // 后 2 个字节填 0x00，用于继续提供 SPI 时钟，把寄存器返回值移出到 MISO。
    uint8_t tx[3] = {(uint8_t)(addr << 1), 0x00, 0x00};

    // 接收数组初始化为 0，便于调试时区分是否真的收到有效返回值。
    uint8_t rx[3] = {0};

    // 选中 ADS8688，开始本次寄存器读取事务。
    ADS8688_CS_LOW();

    // 发送读寄存器命令并同步接收返回数据。
    HAL_SPI_TransmitReceive(spi, tx, rx, 3, 100);

    // 事务结束，释放片选。
    ADS8688_CS_HIGH();

    // 按当前协议实现，真正的寄存器数据出现在第 3 个接收字节 rx[2] 中。
    // 前面的字节通常对应命令阶段或流水线延迟阶段，因此不作为最终返回值。
    return rx[2];
}

uint16_t DT35::ADS8688_ReadADC(void)
{
    // 读取 ADC 结果时，同样通过 4 字节 SPI 事务向芯片提供时钟。
    // 这里发送的 4 个字节全为 0，本质上是“读数占位帧”。
    uint8_t tx[4] = {0};

    // 接收缓冲区用于保存 ADS8688 回传的转换结果。
    uint8_t rx[4] = {0};

    // 拉低片选，开始一次 ADC 数据读取事务。
    ADS8688_CS_LOW();

    // 通过全双工传输，把本次 ADC 结果从芯片移出。
    HAL_SPI_TransmitReceive(spi, tx, rx, 4, 100);

    // 结束事务。
    ADS8688_CS_HIGH();

    // 当前实现约定：16 位 ADC 数据位于返回帧的后 2 个字节。
    // rx[2] 是高 8 位，rx[3] 是低 8 位，因此需要先左移再拼接。
    return ((uint16_t)rx[2] << 8) | rx[3];
}

/* ======================== ADS8688 Init ==================================== */

void DT35::ADS8688_Init(void)
{
    // 初始化开始前先确保片选处于非选中状态，避免芯片误认为有残留 SPI 事务。
    ADS8688_CS_HIGH();

    /* 硬件复位阶段 */
    // 先把复位脚拉低，强制 ADS8688 回到硬件复位状态。
    ADS8688_RST_LOW();

    // 保持一个极短延时，给芯片足够的低电平复位时间。
    HAL_Delay(1);

    // 释放复位脚，让芯片重新启动。
    ADS8688_RST_HIGH();

    // 再等待一小段时间，确保芯片已经退出硬件复位并稳定。
    HAL_Delay(1);

    /* 软件复位阶段 */
    // 发送软件复位命令，进一步保证内部寄存器和状态机回到可预测状态。
    ADS8688_WriteCmd(ADS8688_CMD_RST);

    // 软件复位后等待较长时间，让芯片内部初始化过程完成。
    HAL_Delay(25);

    /* 通道使能配置阶段 */
    // 将自动序列寄存器写为 0x01：
    // 以当前代码语义看，表示只把 CH0 加入自动序列/有效通道集合。
    ADS8688_WriteReg(ADS8688_REG_AUTO_SEQ_EN, 0x01);

    // 将通道掉电寄存器写为 0xFE：
    // 以当前代码注释目标和实际使用方式来看，表示仅保留 CH0 工作，其余通道关闭或不参与使用。
    ADS8688_WriteReg(ADS8688_REG_CH_PD, 0xFE);

    /* 量程配置阶段 */
    // 把 CH0 的输入量程配置为 0 ~ 10.24V。
    // 这样既覆盖 DT35 的 0 ~ 10V 输出范围，也与后续电压换算常量 ADS8688_FS_VOLTAGE 对应。
    ADS8688_WriteReg(ADS8688_REG_CHIR_0, ADS8688_RANGE_0_10V24);

    // 等待量程配置生效，避免紧接着读寄存器或采样时状态尚未稳定。
    HAL_Delay(10);

    /* 配置回读校验阶段 */
    // 某些现场环境里，SPI 上电初期可能存在偶发配置未写成功的情况。
    // 这里通过回读 AUTO_SEQ_EN 寄存器确认“只启用 CH0”的配置是否真正写入。
    uint8_t retry = 0;

    // 如果回读结果不是期望值 0x01，就重新写入关键寄存器并重试。
    while (ADS8688_ReadReg(ADS8688_REG_AUTO_SEQ_EN) != 0x01) {
        // 重新写入通道自动序列配置。
        ADS8688_WriteReg(ADS8688_REG_AUTO_SEQ_EN, 0x01);

        // 同时重新写入通道掉电配置，保证初始化结果完整一致。
        ADS8688_WriteReg(ADS8688_REG_CH_PD, 0xFE);

        // 每次重试后稍等一段时间，再进行下一轮回读。
        HAL_Delay(50);

        // 为避免异常情况下死循环，最多重试 10 次。
        if (++retry > 10) break;
    }

    /* 手动通道选择阶段 */
    // 发送手动 CH0 命令，明确后续采样固定读取通道 0。
    ADS8688_WriteCmd(ADS8688_CMD_MAN_CH0);

    // 切换手动通道后稍作等待，让内部采样路径稳定。
    HAL_Delay(1);

    /* 首次采样丢弃阶段 */
    // 某些 ADC 或多路切换器在切换模式/通道后的第一帧数据可能仍带有旧状态残留。
    // 因此这里主动读取一次并丢弃，不把它作为正式测量结果。
    ADS8688_ReadADC();
}

/* ======================== Public ========================================= */

void DT35::init(SPI_HandleTypeDef *hspi)
{
    // 保存由外部传入的 SPI 句柄。
    // 后续所有 ADS8688 的寄存器访问和 ADC 读取都依赖这个句柄。
    spi = hspi;

    // 初始化 distance_mm 为 0，表示系统刚启动时还没有真实测距结果。
    data.distance_mm = 0.0f;

    // 初始化 voltage_V 为 0，表示还没有完成任何有效电压采样。
    data.voltage_V   = 0.0f;

    // 初始化 adc_raw 为 0，便于后续判断是否已经发生过采样。
    data.adc_raw     = 0;

    // 初始化 valid 为 0，明确告诉外部：当前 data 内容只是默认值，还不是有效测量结果。
    data.valid       = 0;

    // 完成 ADS8688 的芯片级初始化。
    ADS8688_Init();
}

void DT35::update(void)
{
    // 从 ADS8688 读取一次最新的 16 位 ADC 原始值。
    uint16_t raw = ADS8688_ReadADC();

    // 保存原始码值，方便外部做调试、标定或故障分析。
    data.adc_raw = raw;

    // 将 16 位原始计数值换算为输入电压：
    // 电压 = 原始码值 * 满量程电压 / ADC 最大码值
    // 这里的满量程电压是 10.24V，对应当前配置的 0 ~ 10.24V 单极性量程。
    data.voltage_V = (float)raw * ADS8688_FS_VOLTAGE / ADS8688_ADC_MAX;

    // 使用局部变量 v 参与限幅和距离换算，避免直接修改原始电压测量结果。
    float v = data.voltage_V;

    // 如果换算值由于噪声、偏差或边界条件小于 0，则强制钳位到 0V。
    if (v < 0.0f)               v = 0.0f;

    // 如果换算值超过 DT35 标称最大输出 10V，则钳位到 10V。
    // 这样可以避免把超范围电压继续线性外推到不合理的距离值。
    if (v > DT35_VOLTAGE_MAX_V) v = DT35_VOLTAGE_MAX_V;

    // 按照 DT35 的线性标定关系计算距离：
    // 0V 对应 50mm，10V 对应 10000mm。
    // 因此任意电压 v 对应的距离 = 起点距离 + v * 总距离跨度 / 总电压跨度。
    data.distance_mm = DT35_DIST_AT_0V_MM
                     + v * (DT35_DIST_AT_10V_MM - DT35_DIST_AT_0V_MM)
                           / DT35_VOLTAGE_MAX_V;

    // 标记当前 data 中已经包含一次完整 update() 产生的有效结果。
    data.valid = 1;
}
