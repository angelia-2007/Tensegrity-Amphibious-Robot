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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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
osThreadId defaultTaskHandle;
osThreadId IMU_TASKHandle;
osThreadId BARO_TASKHandle;
osThreadId SBUS_TASKHandle;
osThreadId LED_TASKHandle;
osThreadId CONTROL_TASKHandle;
osThreadId RC_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void imu_(void const * argument);
void baro_(void const * argument);
void sbus_(void const * argument);
void led_(void const * argument);
void control_(void const * argument);
void rc_(void const * argument);

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

  /* definition and creation of IMU_TASK */
  osThreadDef(IMU_TASK, imu_, osPriorityRealtime, 0, 128);
  IMU_TASKHandle = osThreadCreate(osThread(IMU_TASK), NULL);

  /* definition and creation of BARO_TASK */
  osThreadDef(BARO_TASK, baro_, osPriorityIdle, 0, 128);
  BARO_TASKHandle = osThreadCreate(osThread(BARO_TASK), NULL);

  /* definition and creation of SBUS_TASK */
  osThreadDef(SBUS_TASK, sbus_, osPriorityIdle, 0, 128);
  SBUS_TASKHandle = osThreadCreate(osThread(SBUS_TASK), NULL);

  /* definition and creation of LED_TASK */
  osThreadDef(LED_TASK, led_, osPriorityIdle, 0, 128);
  LED_TASKHandle = osThreadCreate(osThread(LED_TASK), NULL);

  /* definition and creation of CONTROL_TASK */
  osThreadDef(CONTROL_TASK, control_, osPriorityIdle, 0, 128);
  CONTROL_TASKHandle = osThreadCreate(osThread(CONTROL_TASK), NULL);

  /* definition and creation of RC_TASK */
  osThreadDef(RC_TASK, rc_, osPriorityIdle, 0, 128);
  RC_TASKHandle = osThreadCreate(osThread(RC_TASK), NULL);

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

/* USER CODE BEGIN Header_imu_ */
/**
* @brief Function implementing the IMU_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_imu_ */
void imu_(void const * argument)
{
  /* USER CODE BEGIN imu_ */
  /* Infinite loop */
  for(;;)
  {
	process_bmi088_data();
	IMU_Update_Mahony(&imu_bmi088_data, 0.002f);//融合上层更新姿态		
	
    osDelay(1);
  }
  /* USER CODE END imu_ */
}

/* USER CODE BEGIN Header_baro_ */
/**
* @brief Function implementing the BARO_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_baro_ */
void baro_(void const * argument)
{
  /* USER CODE BEGIN baro_ */
  /* Infinite loop */
  for(;;)
  {
	Baro.Org_Alt=SPL06_Get_Altitude(); 
	TASK_QMC5883L();
    osDelay(100);
  }
  /* USER CODE END baro_ */
}

/* USER CODE BEGIN Header_sbus_ */
/**
* @brief Function implementing the SBUS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sbus_ */
void sbus_(void const * argument)
{
  /* USER CODE BEGIN sbus_ */
  /* Infinite loop */
  for(;;)
  {	
    osDelay(10);
  }
  /* USER CODE END sbus_ */
}

/* USER CODE BEGIN Header_led_ */
/**
* @brief Function implementing the LED_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_ */
void led_(void const * argument)
{
  /* USER CODE BEGIN led_ */
  /* Infinite loop */
  for(;;)
  {
//	static bool is = true;
//	  if(is)
//	  {
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
//		  is = false;
//	  }else
//	  {
//	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);	 
//			is = true;
//	  }
	   
	LED_Run();
    osDelay(20);
  }
  /* USER CODE END led_ */
}

/* USER CODE BEGIN Header_control_ */
/**
* @brief Function implementing the CONTROL_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_control_ */
void control_(void const * argument)
{
  /* USER CODE BEGIN control_ */
  /* Infinite loop */
  for(;;)
  {
 tense_update();
//	demo_vis();
	osDelay(2);
  }
  /* USER CODE END control_ */
}

/* USER CODE BEGIN Header_rc_ */
/**
* @brief Function implementing the RC_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rc_ */
void rc_(void const * argument)
{
  /* USER CODE BEGIN rc_ */
  /* Infinite loop */
  for(;;)
  {
	//FrSky_INFO_Handle();
    osDelay(50);
  }
  /* USER CODE END rc_ */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
