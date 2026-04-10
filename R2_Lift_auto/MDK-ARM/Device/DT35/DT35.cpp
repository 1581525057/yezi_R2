#include "DT35.h"

DT35 dt35;

/* ======================== GPIO Helpers =================================== */
#define ADS8688_CS_LOW()    HAL_GPIO_WritePin(ADS8688_CS_PORT,  ADS8688_CS_PIN,  GPIO_PIN_RESET)
#define ADS8688_CS_HIGH()   HAL_GPIO_WritePin(ADS8688_CS_PORT,  ADS8688_CS_PIN,  GPIO_PIN_SET)
#define ADS8688_RST_LOW()   HAL_GPIO_WritePin(ADS8688_RST_PORT, ADS8688_RST_PIN, GPIO_PIN_RESET)
#define ADS8688_RST_HIGH()  HAL_GPIO_WritePin(ADS8688_RST_PORT, ADS8688_RST_PIN, GPIO_PIN_SET)

/* ======================== ADS8688 SPI Protocol ============================ */

void DT35::ADS8688_WriteCmd(uint16_t cmd)
{
    // 命令帧：4字节全双工。
    // tx[0..1] = 16位命令字（高字节先）；tx[2..3] = 0x00 dummy
    // rx[2..3] = 芯片同步返回上一次通道的ADC旧数据（流水线）
    uint8_t tx[4] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF), 0x00, 0x00};
    uint8_t rx[4] = {0};

    ADS8688_CS_LOW();
    HAL_SPI_TransmitReceive(spi, tx, rx, 4, 100);
    ADS8688_CS_HIGH();

    last_cmd_adc = ((uint16_t)rx[2] << 8) | rx[3];
}

void DT35::ADS8688_WriteReg(uint8_t addr, uint8_t val)
{
    // 寄存器写：2字节。
    // 第1字节 = (addr << 1) | 0x01（写操作标志）；第2字节 = 数据
    uint8_t tx[2] = {(uint8_t)((addr << 1) | 0x01), val};

    ADS8688_CS_LOW();
    HAL_SPI_Transmit(spi, tx, 2, 100);
    ADS8688_CS_HIGH();
}

uint8_t DT35::ADS8688_ReadReg(uint8_t addr)
{
    // 寄存器读：3字节事务（对齐参考程序）。
    // 第1字节 = addr << 1（读操作，最低位=0）；后2字节 dummy
    // 数据返回在 rx[2]
    uint8_t tx[3] = {(uint8_t)(addr << 1), 0x00, 0x00};
    uint8_t rx[3] = {0};

    ADS8688_CS_LOW();
    HAL_SPI_TransmitReceive(spi, tx, rx, 3, 100);
    ADS8688_CS_HIGH();

    return rx[2];
}

uint16_t DT35::ADS8688_ReadADC(void)
{
    // 手动模式 NO_OP 帧：发4字节全0，芯片将当前通道采样结果移出。
    // 对齐参考程序 Get_MAN_CH_Mode_Data()：
    //   Write(0x00); Write(0x00); datah = Read(); datal = Read();
    // rx[2]=高8位，rx[3]=低8位
    uint8_t tx[4] = {0};
    uint8_t rx[4] = {0};

    ADS8688_CS_LOW();
    HAL_SPI_TransmitReceive(spi, tx, rx, 4, 100);
    ADS8688_CS_HIGH();

    return ((uint16_t)rx[2] << 8) | rx[3];
}

/* ======================== ADS8688 Init ==================================== */

void DT35::ADS8688_Init(void)
{
    ADS8688_CS_HIGH();

    /* 硬件复位 */
    ADS8688_RST_LOW();
    HAL_Delay(1);
    ADS8688_RST_HIGH();
    HAL_Delay(1);

    /* 软件复位 */
    ADS8688_WriteCmd(ADS8688_CMD_RST);
    HAL_Delay(25);

    /* 通道使能：CH0 + CH1 */
    // AUTO_SEQ_EN bit0=CH0, bit1=CH1 → 0x03
    ADS8688_WriteReg(ADS8688_REG_AUTO_SEQ_EN, 0x03);
    // CH_PD：bit=1 掉电，bit=0 工作。0xFC = bit0/bit1 工作，其余掉电
    ADS8688_WriteReg(ADS8688_REG_CH_PD, 0xFC);

    /* 量程：CH0 和 CH1 均配置为 0~10.24V */
    ADS8688_WriteReg(ADS8688_REG_CHIR_0, ADS8688_RANGE_0_10V24);
    ADS8688_WriteReg(ADS8688_REG_CHIR_1, ADS8688_RANGE_0_10V24);
    HAL_Delay(10);

    /* 回读校验 */
    uint8_t retry = 0;
    while (ADS8688_ReadReg(ADS8688_REG_AUTO_SEQ_EN) != 0x03) {
        ADS8688_WriteReg(ADS8688_REG_AUTO_SEQ_EN, 0x03);
        ADS8688_WriteReg(ADS8688_REG_CH_PD, 0xFC);
        HAL_Delay(50);
        if (++retry > 10) break;
    }

    /* 切手动CH0，清空流水线 */
    ADS8688_WriteCmd(ADS8688_CMD_MAN_CH0);
    HAL_Delay(1);
    ADS8688_ReadADC();  // 丢弃：命令切换后第一帧 NO_OP 才把流水线冲干净
}

/* ======================== Public ========================================= */

void DT35::init(SPI_HandleTypeDef *hspi)
{
    spi = hspi;

    ch0 = {0.0f, 0.0f, 0, 0};
    ch1 = {0.0f, 0.0f, 0, 0};

    ADS8688_Init();
}

static void convert_channel(uint16_t raw, DT35_Data_t &out)
{
    out.adc_raw   = raw;
    out.voltage_V = (float)raw * ADS8688_FS_VOLTAGE / ADS8688_ADC_MAX;

    float v = out.voltage_V;
    if (v < 0.0f)               v = 0.0f;
    if (v > DT35_VOLTAGE_MAX_V) v = DT35_VOLTAGE_MAX_V;

    out.distance_mm = DT35_DIST_AT_0V_MM
                    + v * (DT35_DIST_AT_10V_MM - DT35_DIST_AT_0V_MM)
                          / DT35_VOLTAGE_MAX_V;
    out.valid = 1;
}

void DT35::update(void)
{
    // 对齐参考程序手动模式时序：
    // MAN_CH_Mode(ch)  → 命令帧，rx[2..3] 是流水线旧数据（丢弃）
    // Get_MAN_CH_Mode_Data() → NO_OP帧，rx[2..3] 是本通道真实采样值

    ADS8688_WriteCmd(ADS8688_CMD_MAN_CH0);
    uint16_t raw0 = ADS8688_ReadADC();
    convert_channel(raw0, ch0);

    ADS8688_WriteCmd(ADS8688_CMD_MAN_CH1);
    uint16_t raw1 = ADS8688_ReadADC();
    convert_channel(raw1, ch1);
}
