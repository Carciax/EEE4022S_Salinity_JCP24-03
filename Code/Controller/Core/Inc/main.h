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
#define SW1_EXTI_IRQn EXTI4_15_IRQn
#define SW2_Pin GPIO_PIN_5
#define SW2_GPIO_Port GPIOB
#define SW2_EXTI_IRQn EXTI4_15_IRQn
#define RS485_UART_Pin GPIO_PIN_6
#define RS485_UART_GPIO_Port GPIOB
#define RS485_DE_Pin GPIO_PIN_7
#define RS485_DE_GPIO_Port GPIOB
#define DIG2_1_Pin GPIO_PIN_8
#define DIG2_1_GPIO_Port GPIOB
#define DIG2_2_Pin GPIO_PIN_9
#define DIG2_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define GPIO_SALINITY GPIOC
#define GPIO_DEPTH GPIOA

#define NUMBER_0_MASK 0x3F
#define NUMBER_1_MASK 0x06
#define NUMBER_2_MASK 0x5B
#define NUMBER_3_MASK 0x4F
#define NUMBER_4_MASK 0x66
#define NUMBER_5_MASK 0x6D
#define NUMBER_6_MASK 0x7D
#define NUMBER_7_MASK 0x07
#define NUMBER_8_MASK 0x7F
#define NUMBER_9_MASK 0x6F

#define DP_MASK 0x80
#define NEGATIVE_SIGN_MASK 0x40

#define LETTER_A_MASK 0x77
#define LETTER_B_MASK 0x7C
#define LETTER_C_MASK 0x39
#define LETTER_D_MASK 0x5E
#define LETTER_E_MASK 0x79
#define LETTER_F_MASK 0x71
#define LETTER_G_MASK 0x3D
#define LETTER_H_MASK 0x76
#define LETTER_I_MASK 0x06
#define LETTER_J_MASK 0x1E
// #define LETTER_K_MASK 0x76
#define LETTER_L_MASK 0x38
// #define LETTER_M_MASK 0x55
#define LETTER_N_MASK 0x54
#define LETTER_O_MASK 0x5C
#define LETTER_P_MASK 0x73
#define LETTER_Q_MASK 0x67
#define LETTER_R_MASK 0x50
#define LETTER_S_MASK 0x6D
#define LETTER_T_MASK 0x78
#define LETTER_U_MASK 0x3E
#define LETTER_V_MASK 0x1C
// #define LETTER_W_MASK 0x3E
// #define LETTER_X_MASK 0x76
#define LETTER_Y_MASK 0x6E
#define LETTER_Z_MASK 0x5B

#define DIGIT_1_MASK 0x600
#define DIGIT_2_MASK 0x500
#define DIGIT_3_MASK 0x300

#define DIGIT_DELAY 1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
