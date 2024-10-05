/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RS485.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    DISPLAY_SALINITY_DEPTH = 0x00,
    DISPLAY_TEMPERATURE = 0x01,
    DISPLAY_DEPTH = 0x02,
    DISPLAY_SALINITY = 0x03,
    DISPLAY_CONDUCTIVITY = 0x04,
    DISPLAY_RESISTANCE = 0x05,
    DISPLAY_CONFIG_ELECTRODE = 0x06,
    DISPLAY_CONFIG_VOLTAGE_START = 0x07,
    DISPLAY_CONFIG_VOLTAGE_END = 0x08,
    DISPLAY_CONFIG_VOLTAGE_STEP = 0x09,
    DISPLAY_CONFIG_ADC_SAMPLES = 0x0a,
    DISPLAY_CONFIG_R1 = 0x0b,
    DISPLAY_CONFIG_SEND = 0x0c,
    DISPLAY_END = 0x0d,
} Display_Mode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint16_t digit_masks[] = {DIGIT_1_MASK, DIGIT_2_MASK, DIGIT_3_MASK};
const uint8_t number_masks[] = {NUMBER_0_MASK, NUMBER_1_MASK, NUMBER_2_MASK, NUMBER_3_MASK, NUMBER_4_MASK, NUMBER_5_MASK, NUMBER_6_MASK, NUMBER_7_MASK, NUMBER_8_MASK, NUMBER_9_MASK};

const uint32_t display_values[][3] = {
    {0, 0, 0}, // unused
    {0xffff0000 | DIGIT_1_MASK | LETTER_T_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_P_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_D_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_P_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_S_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_A_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_L_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_C_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_N_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_D_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_R_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_S_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_L_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_E_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_V_MASK | DECIMAL_POINT, 0xffff0000 | DIGIT_2_MASK | LETTER_S_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_T_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_V_MASK | DECIMAL_POINT, 0xffff0000 | DIGIT_2_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_D_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_V_MASK | DECIMAL_POINT, 0xffff0000 | DIGIT_2_MASK | LETTER_S_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_P_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_S_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_P_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_L_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_R_MASK, 0xffff0000 | DIGIT_2_MASK | NUMBER_1_MASK, 0xffff0000 | DIGIT_3_MASK | 0},
    {0xffff0000 | DIGIT_1_MASK | LETTER_C_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_F_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_G_MASK},
};

uint32_t salinity_dma_buff[] = {0xffff0000 | DIGIT_1_MASK | NEGATIVE_SIGN_MASK, 0xffff0000 | DIGIT_2_MASK | NEGATIVE_SIGN_MASK, 0xffff0000 | DIGIT_3_MASK | NEGATIVE_SIGN_MASK};
uint32_t depth_dma_buff[] = {0xffff0000 | DIGIT_1_MASK | NEGATIVE_SIGN_MASK, 0xffff0000 | DIGIT_2_MASK | NEGATIVE_SIGN_MASK, 0xffff0000 | DIGIT_3_MASK | NEGATIVE_SIGN_MASK};

Display_Mode display_mode = DISPLAY_SALINITY_DEPTH;
RS485_Expecting expecting_response = EXPECTING_NONE;
uint8_t rx_buffer[DATA_PACKET_SIZE];
uint8_t temperature[DATA_PACKET_SIZE] = {0, 0, 0},
    depth[DATA_PACKET_SIZE] = {0, 0, 0},
    salinity[DATA_PACKET_SIZE] = {0, 0, 0},
    resistance[DATA_PACKET_SIZE] = {0, 0, 0},
    conductivity[DATA_PACKET_SIZE] = {0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void update_buffer(uint32_t *buffer, uint8_t *data);
void rs485_transmit(uint8_t *data, uint16_t size);
void rs485_receive_IT(uint16_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI2_Init();
    MX_USART1_UART_Init();
    MX_TIM6_Init();
    MX_TIM17_Init();

    /* Initialize interrupts */
    MX_NVIC_Init();
    /* USER CODE BEGIN 2 */
    HAL_DMA_Start(&hdma_tim6_up, (uint32_t)salinity_dma_buff, (uint32_t) & (GPIO_SALINITY->BSRR), 3);
    HAL_TIM_Base_Start(&htim6);
    __HAL_TIM_ENABLE_DMA(&htim6, TIM_DMA_UPDATE);

    HAL_DMA_Start(&hdma_tim17_ch1_up, (uint32_t)depth_dma_buff, (uint32_t) & (GPIO_DEPTH->BSRR), 3);
    HAL_TIM_Base_Start(&htim17);
    __HAL_TIM_ENABLE_DMA(&htim17, TIM_DMA_CC1);

    // uint8_t command[] = {COMMAND_RESET};
    // rs485_transmit(command, 1);
    /* USER CODE END 2 */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void)
{
    /* TIM6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_IRQn);
    /* USART1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    /* EXTI4_15_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
        switch (expecting_response)
        {
        case EXPECTING_SALINITY_DEPTH:
            salinity[0] = rx_buffer[0];
            salinity[1] = rx_buffer[1];
            salinity[2] = rx_buffer[2];
            if (display_mode == DISPLAY_SALINITY_DEPTH)
            {
                update_buffer(salinity_dma_buff, salinity);
            }

            expecting_response = EXPECTING_DEPTH;
            uint8_t command[] = {COMMAND_GET_DEPTH};
            rs485_transmit(command, 1);
            rs485_receive_IT(DATA_PACKET_SIZE);
            break;
        case EXPECTING_TEMPERATURE:
            temperature[0] = rx_buffer[0];
            temperature[1] = rx_buffer[1];
            temperature[2] = rx_buffer[2];
            if (display_mode == DISPLAY_TEMPERATURE)
            {
                update_buffer(depth_dma_buff, temperature);
            }
            break;
        case EXPECTING_DEPTH:
            depth[0] = rx_buffer[0];
            depth[1] = rx_buffer[1];
            depth[2] = rx_buffer[2];
            if (display_mode == DISPLAY_DEPTH)
            {
                update_buffer(depth_dma_buff, depth);
            }

            break;
        case EXPECTING_SALINITY:
            salinity[0] = rx_buffer[0];
            salinity[1] = rx_buffer[1];
            salinity[2] = rx_buffer[2];
            if (display_mode == DISPLAY_SALINITY)
            {
                update_buffer(depth_dma_buff, salinity);
            }

            break;
        case EXPECTING_CONDUCTIVITY:
            conductivity[0] = rx_buffer[0];
            conductivity[1] = rx_buffer[1];
            conductivity[2] = rx_buffer[2];
            if (display_mode == DISPLAY_CONDUCTIVITY)
            {
                update_buffer(depth_dma_buff, conductivity);
            }

            break;
        case EXPECTING_RESISTANCE:
            resistance[0] = rx_buffer[0];
            resistance[1] = rx_buffer[1];
            resistance[2] = rx_buffer[2];
            if (display_mode == DISPLAY_RESISTANCE)
            {
                update_buffer(depth_dma_buff, resistance);
            }

            break;
        default:
            break;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t command[] = {0};
    switch (GPIO_Pin)
    {
    case SW1_Pin:
        switch (display_mode)
        {
        case DISPLAY_SALINITY_DEPTH:
            expecting_response = EXPECTING_SALINITY_DEPTH;
            command[0] = COMMAND_GET_SALINITY_DEPTH;
            rs485_transmit(command, 1);
            rs485_receive_IT(DATA_PACKET_SIZE);
            break;
        case DISPLAY_TEMPERATURE:
            expecting_response = EXPECTING_TEMPERATURE;
            command[0] = COMMAND_GET_TEMPERATURE;
            rs485_transmit(command, 1);
            rs485_receive_IT(DATA_PACKET_SIZE);
            break;
        case DISPLAY_DEPTH:
            expecting_response = EXPECTING_DEPTH;
            command[0] = COMMAND_GET_DEPTH;
            rs485_transmit(command, 1);
            rs485_receive_IT(DATA_PACKET_SIZE);
            break;
        case DISPLAY_SALINITY:
            expecting_response = EXPECTING_SALINITY;
            command[0] = COMMAND_GET_SALINITY;
            rs485_transmit(command, 1);
            rs485_receive_IT(DATA_PACKET_SIZE);
            break;
        case DISPLAY_CONDUCTIVITY:
            expecting_response = EXPECTING_CONDUCTIVITY;
            command[0] = COMMAND_GET_CONDUCTIVITY;
            rs485_transmit(command, 1);
            rs485_receive_IT(DATA_PACKET_SIZE);
            break;
        case DISPLAY_RESISTANCE:
            expecting_response = EXPECTING_RESISTANCE;
            command[0] = COMMAND_GET_RESISTANCE;
            rs485_transmit(command, 1);
            rs485_receive_IT(DATA_PACKET_SIZE);
            break;
        default:
            break;
        }
        break;
    case SW2_Pin:
        display_mode = (display_mode + 1) % DISPLAY_END;
        salinity_dma_buff[0] = display_values[display_mode][0];
        salinity_dma_buff[1] = display_values[display_mode][1];
        salinity_dma_buff[2] = display_values[display_mode][2];
        switch (display_mode)
        {
        case DISPLAY_SALINITY_DEPTH:
            update_buffer(salinity_dma_buff, salinity);
            update_buffer(depth_dma_buff, depth);
            break;
        case DISPLAY_TEMPERATURE:
            update_buffer(depth_dma_buff, temperature);
            break;
        case DISPLAY_DEPTH:
            update_buffer(depth_dma_buff, depth);
            break;
        case DISPLAY_SALINITY:
            update_buffer(depth_dma_buff, salinity);
            break;
        case DISPLAY_CONDUCTIVITY:
            update_buffer(depth_dma_buff, conductivity);
            break;
        case DISPLAY_RESISTANCE:
            update_buffer(depth_dma_buff, resistance);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

void update_buffer(uint32_t *buffer, uint8_t *data)
{
    for (uint8_t i = 0; i < DATA_PACKET_SIZE; i++)
    {
        if (data[i] & NEGATIVE_SIGN)
        {
            buffer[i] = (0xffff << 16) | digit_masks[i] | NEGATIVE_SIGN_MASK;
        }
        else
        {
            buffer[i] = (0xffff << 16) | digit_masks[i] | number_masks[data[i] & 0x0f];
            if (data[i] & DECIMAL_POINT)
            {
                buffer[i] |= DP_MASK;
            }
        }
    }
}

void rs485_transmit(uint8_t *data, uint16_t size)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    if (HAL_HalfDuplex_EnableTransmitter(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UART_Transmit(&huart1, data, size, 1000) != HAL_OK)
    {
        Error_Handler();
    }
}

void rs485_receive_IT(uint16_t size)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    if (HAL_HalfDuplex_EnableReceiver(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UART_Receive_IT(&huart1, rx_buffer, size) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
