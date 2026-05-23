#include "main.h"
#include "GripPush.h"
#include "cmsis_os.h"

/**
 * @brief 控制夹爪打开
 *
 * @return 无
 */
void claw_open(void)
{
    HAL_GPIO_WritePin(CLAW_GPIO_Port, CLAW_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 控制夹爪关闭
 *
 * @return 无
 */
void claw_close(void)
{
    HAL_GPIO_WritePin(CLAW_GPIO_Port, CLAW_Pin, GPIO_PIN_SET);
}
