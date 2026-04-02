/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for CHASSIS_TASK */
osThreadId_t CHASSIS_TASKHandle;
uint32_t CHASSIS_TASKBuffer[ 512 ];
osStaticThreadDef_t CHASSIS_TASKControlBlock;
const osThreadAttr_t CHASSIS_TASK_attributes = {
  .name = "CHASSIS_TASK",
  .cb_mem = &CHASSIS_TASKControlBlock,
  .cb_size = sizeof(CHASSIS_TASKControlBlock),
  .stack_mem = &CHASSIS_TASKBuffer[0],
  .stack_size = sizeof(CHASSIS_TASKBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for USART_TASK */
osThreadId_t USART_TASKHandle;
uint32_t USART_TASKBuffer[ 512 ];
osStaticThreadDef_t USART_TASKControlBlock;
const osThreadAttr_t USART_TASK_attributes = {
  .name = "USART_TASK",
  .cb_mem = &USART_TASKControlBlock,
  .cb_size = sizeof(USART_TASKControlBlock),
  .stack_mem = &USART_TASKBuffer[0],
  .stack_size = sizeof(USART_TASKBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LIFT_TASK */
osThreadId_t LIFT_TASKHandle;
uint32_t LIFT_TASKBuffer[ 512 ];
osStaticThreadDef_t LIFT_TASKControlBlock;
const osThreadAttr_t LIFT_TASK_attributes = {
  .name = "LIFT_TASK",
  .cb_mem = &LIFT_TASKControlBlock,
  .cb_size = sizeof(LIFT_TASKControlBlock),
  .stack_mem = &LIFT_TASKBuffer[0],
  .stack_size = sizeof(LIFT_TASKBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PLAN_ROUTE */
osThreadId_t PLAN_ROUTEHandle;
uint32_t PLAN_ROUTEBuffer[ 512 ];
osStaticThreadDef_t PLAN_ROUTEControlBlock;
const osThreadAttr_t PLAN_ROUTE_attributes = {
  .name = "PLAN_ROUTE",
  .cb_mem = &PLAN_ROUTEControlBlock,
  .cb_size = sizeof(PLAN_ROUTEControlBlock),
  .stack_mem = &PLAN_ROUTEBuffer[0],
  .stack_size = sizeof(PLAN_ROUTEBuffer),
  .priority = (osPriority_t) osPriorityNormal2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void chassis_task(void *argument);
void usart_task(void *argument);
void lift_task(void *argument);
void plan_route(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CHASSIS_TASK */
  CHASSIS_TASKHandle = osThreadNew(chassis_task, NULL, &CHASSIS_TASK_attributes);

  /* creation of USART_TASK */
  USART_TASKHandle = osThreadNew(usart_task, NULL, &USART_TASK_attributes);

  /* creation of LIFT_TASK */
  LIFT_TASKHandle = osThreadNew(lift_task, NULL, &LIFT_TASK_attributes);

  /* creation of PLAN_ROUTE */
  PLAN_ROUTEHandle = osThreadNew(plan_route, NULL, &PLAN_ROUTE_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_chassis_task */
/**
  * @brief  Function implementing the CHASSIS_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_chassis_task */
__weak void chassis_task(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN chassis_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END chassis_task */
}

/* USER CODE BEGIN Header_usart_task */
/**
* @brief Function implementing the USART_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usart_task */
__weak void usart_task(void *argument)
{
  /* USER CODE BEGIN usart_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usart_task */
}

/* USER CODE BEGIN Header_lift_task */
/**
* @brief Function implementing the LIFT_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lift_task */
__weak void lift_task(void *argument)
{
  /* USER CODE BEGIN lift_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END lift_task */
}

/* USER CODE BEGIN Header_plan_route */
/**
* @brief Function implementing the PLAN_ROUTE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_plan_route */
__weak void plan_route(void *argument)
{
  /* USER CODE BEGIN plan_route */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END plan_route */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

