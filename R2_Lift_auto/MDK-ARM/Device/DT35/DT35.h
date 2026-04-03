#ifndef DT35_H
#define DT35_H

#include "main.h"

/*
 * DT35 模块说明：
 * 1. DT35 激光位移传感器输出的是模拟电压信号，本模块不直接和 DT35 做数字通信。
 * 2. 模拟电压先进入 ADS8688 模数转换器，再由 STM32 通过 SPI 读取转换结果。
 * 3. 当前实现只使用 ADS8688 的 CH0 通道，并把该通道配置为 0 ~ 10.24V 单极性量程。
 * 4. 读取到的 ADC 原始值会先换算成实际电压，再按 DT35 的量程关系线性换算为毫米距离。
 * 5. 这里的注释以“当前代码实际行为”为准，便于后续维护和排查，不主动推翻既有配置。
 */

/* ======================== ADS8688 Command Register ======================== */
#define ADS8688_CMD_NO_OP       0x0000  // 空操作命令：本次 SPI 事务不触发额外控制动作，常用于占位或维持时序。
#define ADS8688_CMD_STDBY       0x8200  // 待机命令：让 ADS8688 进入待机状态，当前模块未实际使用。
#define ADS8688_CMD_PWR_DN      0x8300  // 掉电命令：让 ADS8688 进入低功耗掉电状态，当前模块未实际使用。
#define ADS8688_CMD_RST         0x8500  // 软件复位命令：用于在上电后通过 SPI 将芯片内部状态恢复到默认状态。
#define ADS8688_CMD_AUTO_RST    0xA000  // 自动扫描复位命令：用于复位自动扫描状态机，当前模块未实际使用。
#define ADS8688_CMD_MAN_CH0     0xC000  // 手动选择 CH0 采样：后续转换固定针对通道 0。
#define ADS8688_CMD_MAN_CH1     0xC400  // 手动选择 CH1 采样：当前模块未使用，仅保留定义。
#define ADS8688_CMD_MAN_CH2     0xC800  // 手动选择 CH2 采样：当前模块未使用，仅保留定义。
#define ADS8688_CMD_MAN_CH3     0xCC00  // 手动选择 CH3 采样：当前模块未使用，仅保留定义。
#define ADS8688_CMD_MAN_CH4     0xD000  // 手动选择 CH4 采样：当前模块未使用，仅保留定义。
#define ADS8688_CMD_MAN_CH5     0xD400  // 手动选择 CH5 采样：当前模块未使用，仅保留定义。
#define ADS8688_CMD_MAN_CH6     0xD800  // 手动选择 CH6 采样：当前模块未使用，仅保留定义。
#define ADS8688_CMD_MAN_CH7     0xDC00  // 手动选择 CH7 采样：当前模块未使用，仅保留定义。
#define ADS8688_CMD_MAN_AUX     0xE000  // 手动选择 AUX 通道采样：当前模块未使用，仅保留定义。

/* ====================== ADS8688 Program Register ========================= */
#define ADS8688_REG_AUTO_SEQ_EN 0x01    // 自动序列使能寄存器：每一位对应一个通道是否参与自动扫描。
#define ADS8688_REG_CH_PD       0x02    // 通道掉电控制寄存器：用于控制各通道上电/掉电状态。
#define ADS8688_REG_FEATURE_SEL 0x03    // 功能选择寄存器：可配置若干芯片特性，当前模块未使用。
#define ADS8688_REG_CHIR_0      0x05    // CH0 输入量程寄存器：配置通道 0 的电压量程。
#define ADS8688_REG_CHIR_1      0x06    // CH1 输入量程寄存器：当前模块未使用。
#define ADS8688_REG_CHIR_2      0x07    // CH2 输入量程寄存器：当前模块未使用。
#define ADS8688_REG_CHIR_3      0x08    // CH3 输入量程寄存器：当前模块未使用。
#define ADS8688_REG_CHIR_4      0x09    // CH4 输入量程寄存器：当前模块未使用。
#define ADS8688_REG_CHIR_5      0x0A    // CH5 输入量程寄存器：当前模块未使用。
#define ADS8688_REG_CHIR_6      0x0B    // CH6 输入量程寄存器：当前模块未使用。
#define ADS8688_REG_CHIR_7      0x0C    // CH7 输入量程寄存器：当前模块未使用。

/* ====================== Input Range Selection ============================ */
#define ADS8688_RANGE_PM_10V24  0x00    // 双极性 +/-10.24V 量程：允许采集正负电压，当前模块未使用。
#define ADS8688_RANGE_PM_5V12   0x01    // 双极性 +/-5.12V 量程：当前模块未使用。
#define ADS8688_RANGE_PM_2V56   0x02    // 双极性 +/-2.56V 量程：当前模块未使用。
#define ADS8688_RANGE_0_10V24   0x05    // 单极性 0 ~ 10.24V 量程：当前 CH0 使用该配置，与 DT35 的 0 ~ 10V 输出匹配。
#define ADS8688_RANGE_0_5V12    0x06    // 单极性 0 ~ 5.12V 量程：当前模块未使用。

/* ====================== DT35 Sensor Parameters =========================== */
#define DT35_VOLTAGE_MAX_V      10.0f     // DT35 在当前应用下的有效满量程输出电压，单位 V，用于距离换算上限。
#define DT35_DIST_AT_0V_MM      50.0f     // 当传感器输出 0V 时，对应的测量距离，单位 mm。
#define DT35_DIST_AT_10V_MM     10000.0f  // 当传感器输出 10V 时，对应的测量距离，单位 mm。

#define ADS8688_FS_VOLTAGE      10.24f    // ADS8688 在当前量程配置下的满量程电压，单位 V。
#define ADS8688_ADC_MAX         65535.0f  // 16 位 ADC 的最大码值，用于把原始计数换算成电压。

/* ====================== CS / RST Pin Definition ========================== */
#define ADS8688_CS_PORT         GPIOE       // ADS8688 片选信号所在的 GPIO 端口。
#define ADS8688_CS_PIN          GPIO_PIN_14 // ADS8688 片选信号引脚，SPI 每次事务前拉低、结束后拉高。
#define ADS8688_RST_PORT        GPIOB       // ADS8688 硬件复位信号所在的 GPIO 端口。
#define ADS8688_RST_PIN         GPIO_PIN_8  // ADS8688 硬件复位引脚，用于在初始化阶段强制复位芯片。

/* ====================== Data Struct ====================================== */
struct DT35_Data_t {
    float    distance_mm;  // 当前换算后的距离值，单位 mm，每次调用 update() 后刷新。
    float    voltage_V;    // 由 ADC 原始值换算得到的输入电压，单位 V，反映 DT35 当前模拟输出。
    uint16_t adc_raw;      // 直接从 ADS8688 读出的 16 位原始采样值，便于调试和标定。
    uint8_t  valid;        // 数据有效标志：0 表示尚未完成有效更新，1 表示至少成功执行过一次 update()。
};

/* ====================== DT35 Class ======================================= */
class DT35 {
public:
    DT35_Data_t data;  // 对外公开的数据缓存区，保存最近一次采样和换算结果。

    /*
     * 初始化 DT35 模块。
     * 参数：
     *   hspi - 已经由外部完成底层初始化的 SPI 句柄，后续所有 ADS8688 通信都通过它完成。
     * 调用时机：
     *   系统 GPIO、SPI 外设准备完毕之后调用一次即可。
     * 主要动作：
     *   1. 保存 SPI 句柄；
     *   2. 清空 data 中的历史值；
     *   3. 完成 ADS8688 的硬复位、软复位和通道量程配置。
     */
    void init(SPI_HandleTypeDef *hspi);

    /*
     * 执行一次采样更新。
     * 主要动作：
     *   1. 从 ADS8688 读取一次 CH0 的 16 位原始值；
     *   2. 把原始值换算为实际电压；
     *   3. 把电压限制到 DT35 的有效输出范围内；
     *   4. 根据线性关系换算出毫米距离；
     *   5. 将 valid 置 1，表示 data 中已有有效结果。
     */
    void update(void);

private:
    SPI_HandleTypeDef *spi;  // 内部保存的 SPI 句柄指针，供所有寄存器读写函数复用。

    /*
     * 完成 ADS8688 的初始化配置。
     * 当前代码的配置目标是：
     *   1. 复位芯片；
     *   2. 只使能 CH0；
     *   3. 将 CH0 配置为 0 ~ 10.24V 输入量程；
     *   4. 切换到手动 CH0 采样模式；
     *   5. 丢弃首次无效或不稳定的采样结果。
     */
    void ADS8688_Init(void);

    /*
     * 向 ADS8688 发送 16 位命令字。
     * 用途：
     *   用于软件复位、切换手动通道、进入待机等命令类操作。
     */
    void ADS8688_WriteCmd(uint16_t cmd);

    /*
     * 向 ADS8688 的程序寄存器写入一个 8 位数值。
     * 参数：
     *   addr - 目标寄存器地址；
     *   val  - 需要写入的寄存器值。
     */
    void ADS8688_WriteReg(uint8_t addr, uint8_t val);

    /*
     * 从 ADS8688 的程序寄存器读取一个 8 位数值。
     * 返回值：
     *   读取到的寄存器内容。
     */
    uint8_t  ADS8688_ReadReg(uint8_t addr);

    /*
     * 从 ADS8688 读取一次 16 位 ADC 转换结果。
     * 返回值：
     *   本次 SPI 事务返回的 16 位采样码值。
     */
    uint16_t ADS8688_ReadADC(void);
};

extern DT35 dt35;  // 全局 DT35 实例，供工程其他模块直接访问。

#endif
