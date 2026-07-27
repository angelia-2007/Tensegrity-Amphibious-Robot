/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "filter.h"
#include "system.h"	
#include "math.h"
#include <arm_math.h>
#include <stdbool.h>
#include "imu_bmi088.h"
#include "soft_iic.h"
#include "spl06.h"
#include "motor.h"
#include "led.h"
#include "motor.h"
#include "I2C.h"
#include "frsky.h"
#include "rc.h"
#include "control.h"
#include "QMC5883L.h"
#include "pid.h"
#include "alt.h"



//STS
#include "INST.h"
#include "SCS.h"
#include "SMS_STS.h"
#include "STS3032.h"
#include "WritePos.h"



extern CAN_HandleTypeDef hcan1;
//extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
//extern TIM_HandleTypeDef htim6;
//extern TIM_HandleTypeDef htim2;
//extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;

extern SPI_HandleTypeDef hspi1;
//extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;

extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern uint8_t SBUS_buff[250];

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOF_CON1_Pin GPIO_PIN_14
#define TOF_CON1_GPIO_Port GPIOC
#define TOF_CON2_Pin GPIO_PIN_15
#define TOF_CON2_GPIO_Port GPIOC
#define IIC_SCL_F4_Pin GPIO_PIN_0
#define IIC_SCL_F4_GPIO_Port GPIOC
#define IIC_SDA_F4_Pin GPIO_PIN_1
#define IIC_SDA_F4_GPIO_Port GPIOC
#define O11_Pin GPIO_PIN_2
#define O11_GPIO_Port GPIOA
#define O12_Pin GPIO_PIN_3
#define O12_GPIO_Port GPIOA
#define O1_Pin GPIO_PIN_6
#define O1_GPIO_Port GPIOA
#define O2_Pin GPIO_PIN_7
#define O2_GPIO_Port GPIOA
#define IO1_Pin GPIO_PIN_4
#define IO1_GPIO_Port GPIOC
#define IO2_Pin GPIO_PIN_5
#define IO2_GPIO_Port GPIOC
#define O3_Pin GPIO_PIN_0
#define O3_GPIO_Port GPIOB
#define O4_Pin GPIO_PIN_1
#define O4_GPIO_Port GPIOB
#define CSB_ACCEL_Pin GPIO_PIN_12
#define CSB_ACCEL_GPIO_Port GPIOB
#define CSB_GYRO_Pin GPIO_PIN_13
#define CSB_GYRO_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_14
#define SDA_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_15
#define SCL_GPIO_Port GPIOB
#define O7_Pin GPIO_PIN_6
#define O7_GPIO_Port GPIOC
#define O8_Pin GPIO_PIN_7
#define O8_GPIO_Port GPIOC
#define O9_Pin GPIO_PIN_8
#define O9_GPIO_Port GPIOC
#define O10_Pin GPIO_PIN_9
#define O10_GPIO_Port GPIOC
#define O5_Pin GPIO_PIN_9
#define O5_GPIO_Port GPIOA
#define O6_Pin GPIO_PIN_10
#define O6_GPIO_Port GPIOA
#define SPI1_CS1_Pin GPIO_PIN_7
#define SPI1_CS1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
