/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "stdarg.h"
#include "string.h"

#include "usart.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "bsp_usart.h"

#include "can_receive_transmit.h"
#include "remote_control.h"
#include "chassis_behaviour.h"
#include "chassis_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId RC_TaskHandle;
osThreadId Chassis_TaskHandle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const RC_ctrl_t *local_rc_ctrl;

void*p=NULL;
void usart_printf(const char *fmt,...)
{
  static uint8_t tx_buf[256] = {0};
  static va_list ap;
  static uint16_t len;
  va_start(ap, fmt);

  //return length of string
  //返回字符串长�?????
  len = vsprintf((char *)tx_buf, fmt, ap);

  va_end(ap);

  HAL_UART_Transmit_DMA(&huart1, tx_buf, len);

}
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId StartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of StartTask */
  osThreadDef(StartTask, Start_Task, osPriorityIdle, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
    {
      osDelay(1);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_Task */
void Start_RC_Task(void const * argument);
void Start_Chassis_Task(void const * argument);
/**
 * @brief Function implementing the StartTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_Task */
void Start_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Task */
  /* Infinite loop */
  taskENTER_CRITICAL();
  Remote_Control_Init();

  local_rc_ctrl = get_remote_control_point();

  osThreadDef(RC_Task, Start_RC_Task, osPriorityRealtime, 0, 128);
  RC_TaskHandle = osThreadCreate(osThread(RC_Task), NULL);

  osThreadDef(Chassis_Task, Start_Chassis_Task, osPriorityNormal, 0, 128);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  osThreadTerminate(StartTaskHandle);

  taskEXIT_CRITICAL();
  /* USER CODE END Start_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Start_RC_Task(void const * argument)
{
  /* USER CODE BEGIN Start_RC_Task */
  uint32_t PreviousWakeTime=osKernelSysTick();
  /* Infinite loop */
  while(1)
    {
      //鼠标暂时无数�????
      osDelayUntil(&PreviousWakeTime,100);
      usart_printf(
	  "**********\r\n\
		ch0:%d\r\n\
		ch1:%d\r\n\
		ch2:%d\r\n\
		ch3:%d\r\n\
		ch4:%d\r\n\
		s1:%d\r\n\
		s2:%d\r\n\
	  	motor1_V:%d\r\n\
	  	motor2_V:%d\r\n\
	  	motor3_V:%d\r\n\
	  	motor4_V:%d\r\n\
	  	motor1_A:%d\r\n\
	  	motor2_A:%d\r\n\
	  	motor3_A:%d\r\n\
	  	motor4_A:%d\r\n\
		**********\r\n",
		local_rc_ctrl->rc.ch[0], local_rc_ctrl->rc.ch[1], local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[3], local_rc_ctrl->rc.ch[4],
		local_rc_ctrl->rc.s[0], local_rc_ctrl->rc.s[1],chassis_move.motor_chassis[0].chassis_motor_measure->speed_rpm,
		chassis_move.motor_chassis[1].chassis_motor_measure->speed_rpm,chassis_move.motor_chassis[2].chassis_motor_measure->speed_rpm,
		chassis_move.motor_chassis[3].chassis_motor_measure->speed_rpm,chassis_move.motor_chassis[0].chassis_motor_measure->given_current,
		chassis_move.motor_chassis[1].chassis_motor_measure->given_current,chassis_move.motor_chassis[2].chassis_motor_measure->given_current,
		chassis_move.motor_chassis[3].chassis_motor_measure->given_current);
    }
  /* USER CODE END Start_RC_Task */
}

//丑陋的写法，忘记怎么把任务独立在一个文件要怎么弄才行
void Start_Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Chassis_Task */
  uint32_t PreviousWakeTime=osKernelSysTick();
  //底盘初始�?
  Chassis_Init(&chassis_move);
  /* Infinite loop */
  while(1)
    {
      osDelayUntil(&PreviousWakeTime,2);
      Chassis_Task(p);
    }
  /* USER CODE END Start_Chassis_Task */
}
/* USER CODE END Application */
