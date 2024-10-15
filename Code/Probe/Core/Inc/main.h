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
#define SW_Shield2_V_Pin GPIO_PIN_13
#define SW_Shield2_V_GPIO_Port GPIOC
#define SW_Shield1_GND_Pin GPIO_PIN_14
#define SW_Shield1_GND_GPIO_Port GPIOC
#define SW_Shield2_GND_Pin GPIO_PIN_15
#define SW_Shield2_GND_GPIO_Port GPIOC
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
#define Au1_Pin GPIO_PIN_4
#define Au1_GPIO_Port GPIOA
#define Au2_Pin GPIO_PIN_5
#define Au2_GPIO_Port GPIOA
#define Ti1_Pin GPIO_PIN_6
#define Ti1_GPIO_Port GPIOA
#define Ti2_Pin GPIO_PIN_7
#define Ti2_GPIO_Port GPIOA
#define SW_R1_Au1_Pin GPIO_PIN_4
#define SW_R1_Au1_GPIO_Port GPIOC
#define SW_R1_Au2_Pin GPIO_PIN_5
#define SW_R1_Au2_GPIO_Port GPIOC
#define Calib_Pin GPIO_PIN_0
#define Calib_GPIO_Port GPIOB
#define LED_Red_Pin GPIO_PIN_1
#define LED_Red_GPIO_Port GPIOB
#define LED_Green_Pin GPIO_PIN_2
#define LED_Green_GPIO_Port GPIOB
#define SW_R1_Ti1_Pin GPIO_PIN_6
#define SW_R1_Ti1_GPIO_Port GPIOC
#define SW_R1_Ti2_Pin GPIO_PIN_7
#define SW_R1_Ti2_GPIO_Port GPIOC
#define SW_Au1_GND_Pin GPIO_PIN_8
#define SW_Au1_GND_GPIO_Port GPIOC
#define SW_Au2_GND_Pin GPIO_PIN_9
#define SW_Au2_GND_GPIO_Port GPIOC
#define SW_Ti1_GND_Pin GPIO_PIN_10
#define SW_Ti1_GND_GPIO_Port GPIOC
#define SW_Ti2_GND_Pin GPIO_PIN_11
#define SW_Ti2_GND_GPIO_Port GPIOC
#define SW_Shield1_V_Pin GPIO_PIN_12
#define SW_Shield1_V_GPIO_Port GPIOC
#define RS485_UART_Pin GPIO_PIN_6
#define RS485_UART_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_7
#define RS485_DE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define Pressure_device_addr (0x6d << 1)

#define Pressure_mem_addr_request 0x0A
#define Pressure_mem_addr_pressure 0x0B
#define Pressure_mem_addr_temperature 0x0F
#define Pressure_mem_addr_status 0x13

#define Pressure_request_pressure 0x6
#define Pressure_request_temperature 0x4

#define Pressure_status_finished 0x1

#define MAX_SAMPLES 1024
#define V_MIN 93
#define V_MAX 806

#define PACKET_START "Packet Start\n"
#define PACKET_END "Packet End\n\n"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
