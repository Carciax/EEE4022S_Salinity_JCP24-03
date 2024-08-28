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
#define SW_Shield__Au__Pin GPIO_PIN_13
#define SW_Shield__Au__GPIO_Port GPIOC
#define SW_Shield__Au_C14_Pin GPIO_PIN_14
#define SW_Shield__Au_C14_GPIO_Port GPIOC
#define SW_Shield__Au_C15_Pin GPIO_PIN_15
#define SW_Shield__Au_C15_GPIO_Port GPIOC
#define SW_R1_100_Pin GPIO_PIN_0
#define SW_R1_100_GPIO_Port GPIOC
#define SW_R1_1k_Pin GPIO_PIN_1
#define SW_R1_1k_GPIO_Port GPIOC
#define SW_R1_10k_Pin GPIO_PIN_2
#define SW_R1_10k_GPIO_Port GPIOC
#define SW_R1_Calib_Pin GPIO_PIN_3
#define SW_R1_Calib_GPIO_Port GPIOC
#define Unbuff_DAC_Pin GPIO_PIN_0
#define Unbuff_DAC_GPIO_Port GPIOA
#define DAC_Pin GPIO_PIN_1
#define DAC_GPIO_Port GPIOA
#define Amp_Pin GPIO_PIN_2
#define Amp_GPIO_Port GPIOA
#define Signal_Pin GPIO_PIN_3
#define Signal_GPIO_Port GPIOA
#define Au__Pin GPIO_PIN_4
#define Au__GPIO_Port GPIOA
#define Au_A5_Pin GPIO_PIN_5
#define Au_A5_GPIO_Port GPIOA
#define Ti__Pin GPIO_PIN_6
#define Ti__GPIO_Port GPIOA
#define Ti_A7_Pin GPIO_PIN_7
#define Ti_A7_GPIO_Port GPIOA
#define SW_R1_Au__Pin GPIO_PIN_4
#define SW_R1_Au__GPIO_Port GPIOC
#define SW_R1_Au_C5_Pin GPIO_PIN_5
#define SW_R1_Au_C5_GPIO_Port GPIOC
#define Calibration_Pin GPIO_PIN_0
#define Calibration_GPIO_Port GPIOB
#define LED_Red_Pin GPIO_PIN_1
#define LED_Red_GPIO_Port GPIOB
#define LED_Green_Pin GPIO_PIN_2
#define LED_Green_GPIO_Port GPIOB
#define SW_R1_Ti__Pin GPIO_PIN_6
#define SW_R1_Ti__GPIO_Port GPIOC
#define SW_R1_Ti_C7_Pin GPIO_PIN_7
#define SW_R1_Ti_C7_GPIO_Port GPIOC
#define SW_Au__GND_Pin GPIO_PIN_8
#define SW_Au__GND_GPIO_Port GPIOC
#define SW_Au__GNDC9_Pin GPIO_PIN_9
#define SW_Au__GNDC9_GPIO_Port GPIOC
#define SW_Ti__GND_Pin GPIO_PIN_10
#define SW_Ti__GND_GPIO_Port GPIOC
#define SW_Ti__GNDC11_Pin GPIO_PIN_11
#define SW_Ti__GNDC11_GPIO_Port GPIOC
#define SW_Shield__Au_C12_Pin GPIO_PIN_12
#define SW_Shield__Au_C12_GPIO_Port GPIOC
#define RS485_UART_Pin GPIO_PIN_6
#define RS485_UART_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_7
#define RS485_DE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
