#include "stm32h7_ws281x.h"

#define STM32H7_WS281X_BITS_PER_PIXEL 24U

static STM32H7_WS281X_Color s_pixels[STM32H7_WS281X_LED_COUNT];

#if STM32H7_WS281X_MODE == STM32H7_WS281X_MODE_TIM_PWM_DMA
#define STM32H7_WS281X_PWM_FRAME_WORDS ((STM32H7_WS281X_LED_COUNT * STM32H7_WS281X_BITS_PER_PIXEL) + STM32H7_WS281X_PWM_RESET_SLOTS)

static uint32_t s_pwm_buf[STM32H7_WS281X_PWM_FRAME_WORDS];
#endif

static uint32_t NsToCycles(uint32_t ns)
{
    return (uint32_t)((((uint64_t)SystemCoreClock) * ns + 999999999ULL) / 1000000000ULL);
}

static uint32_t UsToCycles(uint32_t us)
{
    return (uint32_t)((((uint64_t)SystemCoreClock) * us + 999999ULL) / 1000000ULL);
}

#if STM32H7_WS281X_MODE == STM32H7_WS281X_MODE_SOFTWARE
static void DwtInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

#if defined(DWT_LAR)
    DWT->LAR = 0xC5ACCE55U;
#endif

    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void DwtDelayCycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
    }
}

static void SoftPinHigh(void)
{
    STM32H7_WS281X_SOFT_GPIO_PORT->BSRR = STM32H7_WS281X_SOFT_GPIO_PIN;
}

static void SoftPinLow(void)
{
    STM32H7_WS281X_SOFT_GPIO_PORT->BSRR = ((uint32_t)STM32H7_WS281X_SOFT_GPIO_PIN << 16U);
}

static void SoftSendBit(uint8_t bit)
{
    static uint32_t t0h_cycles = 0;
    static uint32_t t1h_cycles = 0;
    static uint32_t bit_cycles = 0;
    uint32_t high_cycles;

    if (bit_cycles == 0U)
    {
        t0h_cycles = NsToCycles(STM32H7_WS281X_T0H_NS);
        t1h_cycles = NsToCycles(STM32H7_WS281X_T1H_NS);
        bit_cycles = NsToCycles(STM32H7_WS281X_BIT_NS);
    }

    high_cycles = bit ? t1h_cycles : t0h_cycles;

    SoftPinHigh();
    DwtDelayCycles(high_cycles);
    SoftPinLow();
    DwtDelayCycles(bit_cycles - high_cycles);
}

static void SoftSendByte(uint8_t value)
{
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        SoftSendBit((value & mask) != 0U);
    }
}

static void SoftSendColor(STM32H7_WS281X_Color color)
{
    /* 当前 24V 六灯一控灯带实测色序为 B-R-G。 */
    SoftSendByte(color.b);
    SoftSendByte(color.r);
    SoftSendByte(color.g);
}
#elif STM32H7_WS281X_MODE == STM32H7_WS281X_MODE_TIM_PWM_DMA
static uint32_t PwmAppendByte(uint32_t index, uint8_t value)
{
    uint8_t mask;

    for (mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        s_pwm_buf[index++] = (value & mask) ? STM32H7_WS281X_PWM_CODE_1 : STM32H7_WS281X_PWM_CODE_0;
    }

    return index;
}

static uint32_t PwmAppendColor(uint32_t index, STM32H7_WS281X_Color color)
{
    /* 当前 24V 六灯一控灯带实测色序为 B-R-G。 */
    index = PwmAppendByte(index, color.b);
    index = PwmAppendByte(index, color.r);
    index = PwmAppendByte(index, color.g);

    return index;
}
#endif

void STM32H7_WS281X_Init(void)
{
#if STM32H7_WS281X_MODE == STM32H7_WS281X_MODE_SOFTWARE
    DwtInit();
    SoftPinLow();
#endif

    STM32H7_WS281X_Clear();
}

uint16_t STM32H7_WS281X_GetLedCount(void)
{
    return STM32H7_WS281X_LED_COUNT;
}

void STM32H7_WS281X_SetPixel(uint16_t index, STM32H7_WS281X_Color color)
{
    if (index >= STM32H7_WS281X_LED_COUNT)
    {
        return;
    }

    s_pixels[index] = color;
}

void STM32H7_WS281X_SetAll(STM32H7_WS281X_Color color)
{
    uint16_t i;

    for (i = 0; i < STM32H7_WS281X_LED_COUNT; i++)
    {
        s_pixels[i] = color;
    }
}

HAL_StatusTypeDef STM32H7_WS281X_Show(void)
{
#if STM32H7_WS281X_MODE == STM32H7_WS281X_MODE_SOFTWARE
    uint32_t primask;
    uint16_t i;

    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0; i < STM32H7_WS281X_LED_COUNT; i++)
    {
        SoftSendColor(s_pixels[i]);
    }

    SoftPinLow();
    DwtDelayCycles(UsToCycles(STM32H7_WS281X_RESET_US));

    if (primask == 0U)
    {
        __enable_irq();
    }

    return HAL_OK;
#elif STM32H7_WS281X_MODE == STM32H7_WS281X_MODE_TIM_PWM_DMA
    uint32_t buf_index = 0;
    uint16_t i;
    HAL_StatusTypeDef status;
    uint32_t frame_us;
    uint32_t frame_ms;

    for (i = 0; i < STM32H7_WS281X_LED_COUNT; i++)
    {
        buf_index = PwmAppendColor(buf_index, s_pixels[i]);
    }

    while (buf_index < STM32H7_WS281X_PWM_FRAME_WORDS)
    {
        s_pwm_buf[buf_index++] = 0U;
    }

    HAL_TIM_PWM_Stop_DMA(&STM32H7_WS281X_TIM_HANDLE, STM32H7_WS281X_TIM_CHANNEL);
    status = HAL_TIM_PWM_Start_DMA(&STM32H7_WS281X_TIM_HANDLE,
                                   STM32H7_WS281X_TIM_CHANNEL,
                                   s_pwm_buf,
                                   (uint16_t)STM32H7_WS281X_PWM_FRAME_WORDS);

    if (status != HAL_OK)
    {
        return status;
    }

    frame_us = (STM32H7_WS281X_PWM_FRAME_WORDS * STM32H7_WS281X_BIT_NS) / 1000U;
    frame_us += STM32H7_WS281X_RESET_US;
    frame_ms = (frame_us / 1000U) + 2U;
    HAL_Delay(frame_ms);

    HAL_TIM_PWM_Stop_DMA(&STM32H7_WS281X_TIM_HANDLE, STM32H7_WS281X_TIM_CHANNEL);
    return HAL_OK;
#else
    return HAL_ERROR;
#endif
}

void STM32H7_WS281X_Clear(void)
{
    STM32H7_WS281X_Color black = {0, 0, 0};

    STM32H7_WS281X_SetAll(black);
}
