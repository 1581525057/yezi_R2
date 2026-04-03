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
    uint8_t tx[4] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF), 0x00, 0x00};
    uint8_t rx[4];
    ADS8688_CS_LOW();
    HAL_SPI_TransmitReceive(spi, tx, rx, 4, 100);
    ADS8688_CS_HIGH();
}

void DT35::ADS8688_WriteReg(uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = {(uint8_t)((addr << 1) | 0x01), val};
    ADS8688_CS_LOW();
    HAL_SPI_Transmit(spi, tx, 2, 100);
    ADS8688_CS_HIGH();
}

uint8_t DT35::ADS8688_ReadReg(uint8_t addr)
{
    uint8_t tx[3] = {(uint8_t)(addr << 1), 0x00, 0x00};
    uint8_t rx[3] = {0};
    ADS8688_CS_LOW();
    HAL_SPI_TransmitReceive(spi, tx, rx, 3, 100);
    ADS8688_CS_HIGH();
    return rx[2];
}

uint16_t DT35::ADS8688_ReadADC(void)
{
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

    /* Hardware reset */
    ADS8688_RST_LOW();
    HAL_Delay(1);
    ADS8688_RST_HIGH();
    HAL_Delay(1);

    /* Software reset */
    ADS8688_WriteCmd(ADS8688_CMD_RST);
    HAL_Delay(25);

    /* Enable CH0 only */
    ADS8688_WriteReg(ADS8688_REG_AUTO_SEQ_EN, 0x01);
    ADS8688_WriteReg(ADS8688_REG_CH_PD, 0xFE);

    /* CH0 range: 0 ~ 10.24V */
    ADS8688_WriteReg(ADS8688_REG_CHIR_0, ADS8688_RANGE_0_10V24);
    HAL_Delay(10);

    /* Verify */
    uint8_t retry = 0;
    while (ADS8688_ReadReg(ADS8688_REG_AUTO_SEQ_EN) != 0x01) {
        ADS8688_WriteReg(ADS8688_REG_AUTO_SEQ_EN, 0x01);
        ADS8688_WriteReg(ADS8688_REG_CH_PD, 0xFE);
        HAL_Delay(50);
        if (++retry > 10) break;
    }

    /* Manual CH0 mode */
    ADS8688_WriteCmd(ADS8688_CMD_MAN_CH0);
    HAL_Delay(1);

    /* Discard first reading */
    ADS8688_ReadADC();
}

/* ======================== Public ========================================= */

void DT35::init(SPI_HandleTypeDef *hspi)
{
    spi = hspi;

    data.distance_mm = 0.0f;
    data.voltage_V   = 0.0f;
    data.adc_raw     = 0;
    data.valid       = 0;

    ADS8688_Init();
}

void DT35::update(void)
{
    uint16_t raw = ADS8688_ReadADC();

    data.adc_raw = raw;
    data.voltage_V = (float)raw * ADS8688_FS_VOLTAGE / ADS8688_ADC_MAX;

    float v = data.voltage_V;
    if (v < 0.0f)              v = 0.0f;
    if (v > DT35_VOLTAGE_MAX_V) v = DT35_VOLTAGE_MAX_V;

    data.distance_mm = DT35_DIST_AT_0V_MM
                     + v * (DT35_DIST_AT_10V_MM - DT35_DIST_AT_0V_MM)
                           / DT35_VOLTAGE_MAX_V;
    data.valid = 1;
}
