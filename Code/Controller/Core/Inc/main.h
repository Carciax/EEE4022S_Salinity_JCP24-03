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
#include "stm32f0xx_hal.h"

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
#define LED_Red_Pin GPIO_PIN_0
#define LED_Red_GPIO_Port GPIOF
#define LED_Green_Pin GPIO_PIN_1
#define LED_Green_GPIO_Port GPIOF
#define Salinity_A_Pin GPIO_PIN_0
#define Salinity_A_GPIO_Port GPIOC
#define Salinity_B_Pin GPIO_PIN_1
#define Salinity_B_GPIO_Port GPIOC
#define Salinity_C_Pin GPIO_PIN_2
#define Salinity_C_GPIO_Port GPIOC
#define Salinity_D_Pin GPIO_PIN_3
#define Salinity_D_GPIO_Port GPIOC
#define Depth_A_Pin GPIO_PIN_0
#define Depth_A_GPIO_Port GPIOA
#define Depth_B_Pin GPIO_PIN_1
#define Depth_B_GPIO_Port GPIOA
#define Depth_C_Pin GPIO_PIN_2
#define Depth_C_GPIO_Port GPIOA
#define Depth_D_Pin GPIO_PIN_3
#define Depth_D_GPIO_Port GPIOA
#define DIG3_1_Pin GPIO_PIN_4
#define DIG3_1_GPIO_Port GPIOF
#define DIG3_2_Pin GPIO_PIN_5
#define DIG3_2_GPIO_Port GPIOF
#define Depth_E_Pin GPIO_PIN_4
#define Depth_E_GPIO_Port GPIOA
#define Depth_F_Pin GPIO_PIN_5
#define Depth_F_GPIO_Port GPIOA
#define Depth_G_Pin GPIO_PIN_6
#define Depth_G_GPIO_Port GPIOA
#define Depth_DP_Pin GPIO_PIN_7
#define Depth_DP_GPIO_Port GPIOA
#define Salinity_E_Pin GPIO_PIN_4
#define Salinity_E_GPIO_Port GPIOC
#define Salinity_F_Pin GPIO_PIN_5
#define Salinity_F_GPIO_Port GPIOC
#define DIG1_1_Pin GPIO_PIN_0
#define DIG1_1_GPIO_Port GPIOB
#define DIG1_2_Pin GPIO_PIN_1
#define DIG1_2_GPIO_Port GPIOB
#define DIG1_4_Pin GPIO_PIN_2
#define DIG1_4_GPIO_Port GPIOB
#define DIG2_4_Pin GPIO_PIN_10
#define DIG2_4_GPIO_Port GPIOB
#define DIG2_8_Pin GPIO_PIN_11
#define DIG2_8_GPIO_Port GPIOB
#define SPI_NSS_Pin GPIO_PIN_12
#define SPI_NSS_GPIO_Port GPIOB
#define Salinity_G_Pin GPIO_PIN_6
#define Salinity_G_GPIO_Port GPIOC
#define Salinity_DP_Pin GPIO_PIN_7
#define Salinity_DP_GPIO_Port GPIOC
#define Salinity_DIG1_Pin GPIO_PIN_8
#define Salinity_DIG1_GPIO_Port GPIOC
#define Salinity_DIG2_Pin GPIO_PIN_9
#define Salinity_DIG2_GPIO_Port GPIOC
#define Depth_DIG1_Pin GPIO_PIN_8
#define Depth_DIG1_GPIO_Port GPIOA
#define Depth_DIG2_Pin GPIO_PIN_9
#define Depth_DIG2_GPIO_Port GPIOA
#define Depth_DIG3_Pin GPIO_PIN_10
#define Depth_DIG3_GPIO_Port GPIOA
#define DIG3_4_Pin GPIO_PIN_6
#define DIG3_4_GPIO_Port GPIOF
#define DIG3_8_Pin GPIO_PIN_7
#define DIG3_8_GPIO_Port GPIOF
#define Salinity_DIG3_Pin GPIO_PIN_10
#define Salinity_DIG3_GPIO_Port GPIOC
#define Card_Detect_Pin GPIO_PIN_2
#define Card_Detect_GPIO_Port GPIOD
#define DIG1_8_Pin GPIO_PIN_3
#define DIG1_8_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_4
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_5
#define SW2_GPIO_Port GPIOB
#define RS485_UART_Pin GPIO_PIN_6
#define RS485_UART_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_7
#define RS485_DE_GPIO_Port GPIOB
#define DIG2_1_Pin GPIO_PIN_8
#define DIG2_1_GPIO_Port GPIOB
#define DIG2_2_Pin GPIO_PIN_9
#define DIG2_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
