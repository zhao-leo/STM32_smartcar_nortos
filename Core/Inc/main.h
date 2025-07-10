/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t g_usart2_receivedata; // The data received from the USART2,also the data from gyroscope
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
#define MotorA_control_1_Pin GPIO_PIN_14
#define MotorA_control_1_GPIO_Port GPIOC
#define MotorA_control_2_Pin GPIO_PIN_15
#define MotorA_control_2_GPIO_Port GPIOC
#define MotorB_control_1_Pin GPIO_PIN_4
#define MotorB_control_1_GPIO_Port GPIOA
#define MotorB_control_2_Pin GPIO_PIN_5
#define MotorB_control_2_GPIO_Port GPIOA
#define EncoderA_1_Pin GPIO_PIN_6
#define EncoderA_1_GPIO_Port GPIOA
#define EncoderA_2_Pin GPIO_PIN_7
#define EncoderA_2_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_12
#define SPI_CS_GPIO_Port GPIOB
#define EncoderB_1_Pin GPIO_PIN_8
#define EncoderB_1_GPIO_Port GPIOA
#define EncoderB_2_Pin GPIO_PIN_9
#define EncoderB_2_GPIO_Port GPIOA
#define MotorA_Pin GPIO_PIN_6
#define MotorA_GPIO_Port GPIOB
#define MotorB_Pin GPIO_PIN_7
#define MotorB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
