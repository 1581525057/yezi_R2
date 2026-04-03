#ifndef DT35_H
#define DT35_H

#include "main.h"

/* ======================== ADS8688 Command Register ======================== */
#define ADS8688_CMD_NO_OP       0x0000
#define ADS8688_CMD_STDBY       0x8200
#define ADS8688_CMD_PWR_DN      0x8300
#define ADS8688_CMD_RST         0x8500
#define ADS8688_CMD_AUTO_RST    0xA000
#define ADS8688_CMD_MAN_CH0     0xC000
#define ADS8688_CMD_MAN_CH1     0xC400
#define ADS8688_CMD_MAN_CH2     0xC800
#define ADS8688_CMD_MAN_CH3     0xCC00
#define ADS8688_CMD_MAN_CH4     0xD000
#define ADS8688_CMD_MAN_CH5     0xD400
#define ADS8688_CMD_MAN_CH6     0xD800
#define ADS8688_CMD_MAN_CH7     0xDC00
#define ADS8688_CMD_MAN_AUX     0xE000

/* ====================== ADS8688 Program Register ========================= */
#define ADS8688_REG_AUTO_SEQ_EN 0x01
#define ADS8688_REG_CH_PD       0x02
#define ADS8688_REG_FEATURE_SEL 0x03
#define ADS8688_REG_CHIR_0      0x05
#define ADS8688_REG_CHIR_1      0x06
#define ADS8688_REG_CHIR_2      0x07
#define ADS8688_REG_CHIR_3      0x08
#define ADS8688_REG_CHIR_4      0x09
#define ADS8688_REG_CHIR_5      0x0A
#define ADS8688_REG_CHIR_6      0x0B
#define ADS8688_REG_CHIR_7      0x0C

/* ====================== Input Range Selection ============================ */
#define ADS8688_RANGE_PM_10V24  0x00  // +/-10.24V
#define ADS8688_RANGE_PM_5V12   0x01  // +/-5.12V
#define ADS8688_RANGE_PM_2V56   0x02  // +/-2.56V
#define ADS8688_RANGE_0_10V24   0x05  // 0 ~ 10.24V
#define ADS8688_RANGE_0_5V12    0x06  // 0 ~ 5.12V

/* ====================== DT35 Sensor Parameters =========================== */
#define DT35_VOLTAGE_MAX_V      10.0f
#define DT35_DIST_AT_0V_MM      50.0f
#define DT35_DIST_AT_10V_MM     10000.0f

#define ADS8688_FS_VOLTAGE      10.24f
#define ADS8688_ADC_MAX         65535.0f

/* ====================== CS / RST Pin Definition ========================== */
#define ADS8688_CS_PORT         GPIOE
#define ADS8688_CS_PIN          GPIO_PIN_14
#define ADS8688_RST_PORT        GPIOB
#define ADS8688_RST_PIN         GPIO_PIN_8

/* ====================== Data Struct ====================================== */
struct DT35_Data_t {
    float    distance_mm;
    float    voltage_V;
    uint16_t adc_raw;
    uint8_t  valid;
};

/* ====================== DT35 Class ======================================= */
class DT35 {
public:
    DT35_Data_t data;

    void init(SPI_HandleTypeDef *hspi);
    void update(void);

private:
    SPI_HandleTypeDef *spi;

    void ADS8688_Init(void);
    void ADS8688_WriteCmd(uint16_t cmd);
    void ADS8688_WriteReg(uint8_t addr, uint8_t val);
    uint8_t  ADS8688_ReadReg(uint8_t addr);
    uint16_t ADS8688_ReadADC(void);
};

extern DT35 dt35;

#endif
