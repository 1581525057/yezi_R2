#ifndef STM32H7_WS281X_CONFIG_H
#define STM32H7_WS281X_CONFIG_H

/* ===================== 用户移植时主要修改本文件 =====================
 * 正常情况下，你只需要改这个文件里的宏定义，不需要改 stm32h7_ws281x.c。
 */

/* 【需要修改 1】选择灯带输出方式。
 * STM32H7_WS281X_MODE_SOFTWARE    : one GPIO pin, DWT cycle delay, no timer.
 * STM32H7_WS281X_MODE_TIM_PWM_DMA : one TIM PWM channel with DMA.
 *
 * 如果你想只接一个 GPIO 口，使用软件模拟 WS281x 时序，选 SOFTWARE。
 * 如果你想使用定时器 PWM + DMA，选 TIM_PWM_DMA。
 */
#define STM32H7_WS281X_MODE STM32H7_WS281X_MODE_TIM_PWM_DMA

/* 【需要修改 2】灯带数量。
 * 当前灯带是 120 颗物理灯珠，6 颗为一组控制，实际控制点数量为 20。
 */
#define STM32H7_WS281X_PHYSICAL_LED_COUNT       120U
#define STM32H7_WS281X_LEDS_PER_CONTROL_POINT   6U
#define STM32H7_WS281X_LED_COUNT                \
    (STM32H7_WS281X_PHYSICAL_LED_COUNT / STM32H7_WS281X_LEDS_PER_CONTROL_POINT)

#if (STM32H7_WS281X_PHYSICAL_LED_COUNT % STM32H7_WS281X_LEDS_PER_CONTROL_POINT) != 0
#error "Physical LED count must be divisible by LEDs per control point."
#endif

/* 【通常不需要修改】WS2812B / WS2811 兼容时序，默认 800 kHz。
 * 如果你的灯带规格书写明 T0H/T1H/bit period 不同，再改下面这些值。
 */
#define STM32H7_WS281X_T0H_NS     350U
#define STM32H7_WS281X_T1H_NS     700U
#define STM32H7_WS281X_BIT_NS     1250U
#define STM32H7_WS281X_RESET_US   80U

/* 【需要修改 3：软件 PWM/GPIO 模式】灯带 DATA 引脚连接的 GPIO。
 * 只有当 STM32H7_WS281X_MODE = STM32H7_WS281X_MODE_SOFTWARE 时使用。
 *
 * 示例：
 *   数据线接 PA0：GPIOA + GPIO_PIN_0
 *   数据线接 PB5：GPIOB + GPIO_PIN_5
 *   数据线接 PC7：GPIOC + GPIO_PIN_7
 *
 * 注意：这个 GPIO 需要在 CubeMX 或你的初始化代码中配置为推挽输出。
 */
#define STM32H7_WS281X_SOFT_GPIO_PORT GPIOA
#define STM32H7_WS281X_SOFT_GPIO_PIN  GPIO_PIN_0

/* 【需要修改 4：硬件 PWM + DMA 模式】定时器 PWM 输出口。
 * 只有当 STM32H7_WS281X_MODE = STM32H7_WS281X_MODE_TIM_PWM_DMA 时使用。
 *
 * 你可以随意换成工程里已有的 TIM PWM 通道，例如：
 *   htim1 + TIM_CHANNEL_1
 *   htim2 + TIM_CHANNEL_3
 *   htim3 + TIM_CHANNEL_1
 *
 * 注意：
 * 1. CubeMX 中该 TIM 通道要配置成 PWM Generation。
 * 2. 该 TIM 通道要开启 DMA。
 * 3. 对应 GPIO 要配置为该 TIM 通道的复用功能 AF。
 */
#define STM32H7_WS281X_TIM_HANDLE htim2
#define STM32H7_WS281X_TIM_CHANNEL TIM_CHANNEL_1

/* 【需要修改 5：硬件 PWM + DMA 模式】定时器周期和 0/1 码占空比。
 *
 * 目标 PWM 频率约为 800 kHz：
 *   TIM_CLK / (PSC + 1) / (ARR + 1) = 800000
 *
 * 如果 ARR = 167，则一个 PWM 周期有 168 个计数：
 *   STM32H7_WS281X_PWM_PERIOD_TICKS = ARR + 1 = 168
 *
 * 0 码一般是约 1/3 周期高电平：
 *   STM32H7_WS281X_PWM_CODE_0 = 168 / 3 = 56
 *
 * 1 码一般是约 2/3 周期高电平：
 *   STM32H7_WS281X_PWM_CODE_1 = 168 * 2 / 3 = 112
 *
 * 如果你原工程已经验证 CODE_0 = 67、CODE_1 = 134 可用，
 * 也可以把下面两个值直接改成 67 和 134。
 */
#define STM32H7_WS281X_PWM_PERIOD_TICKS 300U
#define STM32H7_WS281X_PWM_CODE_0       84U
#define STM32H7_WS281X_PWM_CODE_1       168U

/* 【一般不需要修改】硬件 PWM + DMA 模式下的复位低电平长度。
 * WS281x 一帧数据发完后，需要保持低电平一段时间作为 reset。
 * 如果你的灯带刷新不稳定，可以适当增大这个值，例如 100 或 120。
 */
#define STM32H7_WS281X_PWM_RESET_SLOTS  80U

#endif
