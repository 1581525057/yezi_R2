#ifndef AS5047_H
#define AS5047_H

#include "main.h"

/*
 * AS5047P 模块说明：
 * 1. AS5047P 是一颗 14 位磁编码器，当前模块通过 SPI 读取其角度和诊断状态。
 * 2. 角度主通道使用 ANGLECOM（动态误差补偿后的角度值），并换算成 0 ~ 360 度。
 * 3. 除角度外，本模块还支持读取错误寄存器 ERRFL 和诊断寄存器 DIAAGC，便于现场调试。
 * 4. ERRFL 为“读后清零”寄存器，因此只在明确需要取证时才主动读取。
 * 5. DIAAGC 可反映磁场过强/过弱、CORDIC 溢出、内部补偿状态以及 AGC 增益值。
 */

/* ======================== AS5047P Register Address ======================= */
#define AS5047P_REG_NOP      0x0000  // NOP 寄存器地址：常用于第二拍把上一拍读取结果从 MISO 移出。
#define AS5047P_REG_ERRFL    0x0001  // 错误寄存器 ERRFL：记录通信和帧错误，读出后芯片内部会自动清零。
#define AS5047P_REG_PROG     0x0003  // 编程控制寄存器：当前模块未使用，仅保留定义。
#define AS5047P_REG_DIAAGC   0x3FFC  // 诊断与自动增益控制寄存器 DIAAGC。
#define AS5047P_REG_MAG      0x3FFD  // 磁场幅值寄存器 MAG：当前模块未使用，仅保留定义。
#define AS5047P_REG_ANGLEUNC 0x3FFE  // 未补偿角度寄存器：当前模块未使用，仅保留定义。
#define AS5047P_REG_ANGLECOM 0x3FFF  // 补偿后角度寄存器：当前角度读取主通道。

/* ======================== ERRFL Bit Definition =========================== */
#define AS5047P_ERRFL_PARERR_BIT   2  // PARERR：主机发送命令的奇偶校验错误。
#define AS5047P_ERRFL_INVCOMM_BIT  1  // INVCOMM：无效命令或非法访问。
#define AS5047P_ERRFL_FRERR_BIT    0  // FRERR：SPI 帧错误。

/* ======================== DIAAGC Bit Definition ========================== */
#define AS5047P_DIAAGC_MAGL_BIT   11  // MAGL：磁场过弱。
#define AS5047P_DIAAGC_MAGH_BIT   10  // MAGH：磁场过强。
#define AS5047P_DIAAGC_COF_BIT     9  // COF：CORDIC overflow，角度结果不可靠。
#define AS5047P_DIAAGC_LF_BIT      8  // LF：内部 offset compensation 完成。
#define AS5047P_DIAAGC_AGC_MASK 0x00FF// AGC[7:0]：自动增益控制值。

/* ======================== CS Pin Definition ============================== */
#define AS5047P_CS_PORT      GPIOE       // AS5047P 片选信号所在 GPIO 端口。
#define AS5047P_CS_PIN       GPIO_PIN_15 // AS5047P 片选信号引脚。

/* ======================== Data Struct ==================================== */
struct AS5047_Data_t {
    float    angle;             // 当前补偿后角度值，单位 degree，范围约为 0 ~ 360。
    uint16_t angle_raw;         // 当前补偿后角度的 14 位原始码值，范围 0 ~ 16383。
    uint8_t  frame_error_flag;  // 最近一次角度读取返回帧中的 EF 标志，1 表示芯片提示上一帧命令存在错误。

    uint16_t errfl_raw;         // 最近一次读取到的 ERRFL 原始 14 位数据。
    uint8_t  errfl_parerr;      // ERRFL.bit2：命令奇偶校验错误标志。
    uint8_t  errfl_invcomm;     // ERRFL.bit1：无效命令错误标志。
    uint8_t  errfl_frerr;       // ERRFL.bit0：SPI 帧错误标志。

    uint16_t diaagc_raw;        // 最近一次读取到的 DIAAGC 原始 14 位数据。
    uint8_t  diaagc_magl;       // DIAAGC.bit11：磁场过弱标志。
    uint8_t  diaagc_magh;       // DIAAGC.bit10：磁场过强标志。
    uint8_t  diaagc_cof;        // DIAAGC.bit9：CORDIC 溢出标志。
    uint8_t  diaagc_lf;         // DIAAGC.bit8：内部补偿完成标志。
    uint8_t  diaagc_agc;        // DIAAGC.bit7:0：自动增益控制值。
};

class AS5047P
{
public:
    AS5047_Data_t data;  // 对外公开的数据缓存区，保存最近一次角度和诊断状态。

    /*
     * 初始化 AS5047P 模块。
     * 参数：
     *   hspi - 已完成底层初始化的 SPI 句柄。
     * 主要动作：
     *   1. 保存 SPI 句柄；
     *   2. 把 data 内的角度、错误和诊断缓存全部清零。
     */
    void init(SPI_HandleTypeDef *hspi);

    /*
     * 更新一次编码器数据。
     * 固定顺序：
     *   1. 读取 ANGLECOM；
     *   2. 刷新 angle_raw 与 angle；
     *   3. 记录 EF 错误标志；
     *   4. 若 EF=1，则主动读取 ERRFL 并缓存；
     *   5. 每次都读取 DIAAGC 并刷新诊断缓存。
     */
    void updata(void);

    /*
     * 主动读取 ERRFL 错误寄存器。
     * 说明：
     *   ERRFL 是读后清零寄存器，因此该函数除返回当前错误信息外，也会刷新 data 中的错误缓存。
     */
    uint16_t readErrorRegister(void);

    /*
     * 主动读取 DIAAGC 诊断寄存器。
     * 说明：
     *   该函数会返回当前 DIAAGC 原始值，并同步刷新 data 中的诊断缓存。
     */
    uint16_t readDiagAgcRegister(void);

private:
    SPI_HandleTypeDef *spi;  // 模块内部保存的 SPI 句柄，所有 16 位收发都依赖它。

    uint16_t AS5047P_SPI_Transfer16(uint16_t txData);
    uint8_t  AS5047P_EvenParity15(uint16_t value);
    uint16_t AS5047P_MakeCommand(uint16_t addr, uint8_t isRead);
    uint16_t AS5047P_ReadRaw(uint16_t addr);
    uint8_t  AS5047P_CheckErrorFlag(uint16_t rawFrame);
    uint16_t AS5047P_GetData14(uint16_t rawFrame);
    uint16_t AS5047P_ReadAngleRaw(void);
    float    AS5047P_ReadAngleDegree(void);

    void AS5047P_UpdateErrflCache(uint16_t errflData);
    void AS5047P_UpdateDiaagcCache(uint16_t diaagcData);
};

extern AS5047P as5047;

#endif
