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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "voltage.h"
#include "salinity.h"
#include "RS485.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    TASK_NONE = 0x00,
    TASK_MEASURE_TEMPERATURE = 0x01,
    TASK_MEASURE_DEPTH = 0x02,
    TASK_MEASURE_SALINITY = 0x03,
} Task;

typedef enum
{
    SAMPLE_VOLTAGE = 0x00,
    SAMPLE_RESISTANCE = 0x01,
    VALUE_CONDUCTIVITY = 0x02,
    VALUE_TEMPERATURE = 0x03,
    VALUE_PRESSURE = 0x04,
    VALUE_SALINITY = 0x05,
} Sample_Type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Task current_task = TASK_NONE;
RS485_Command command = 0;
RS485_Status status = STATUS_IDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void rs485_transmit(uint8_t *data, uint16_t size);
void rs485_receive_IT(void);
void transmit_sample_data_readable(void *samples, uint16_t num_samples, Sample_Type sample_type);
void transmit_sample_data_binary(void *samples, uint16_t num_samples, Sample_Type sample_type);
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
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART6_UART_Init();
    MX_TIM10_Init();
    /* USER CODE BEGIN 2 */

    voltage_init();
    // VoltageSample_TypeDef samples[10];
    // measure_voltage_sweep(samples, BIDIRECTIONAL, R1_100, Ti, 0, 1 << 9, 10, 2);
    // salinity = calculate_salinity(4.0, 15.0, 100.0);

    // double temperature = measure_temperature();
    // double pressure = measure_pressure();

    HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        HAL_UART_Transmit(&huart6, (uint8_t *)PACKET_START, sizeof(PACKET_START), 1000);

        VoltageSample_TypeDef votlage_samples[NUM_SAMPLES];
        measure_voltage_sweep(votlage_samples, BIDIRECTIONAL, R1_100, Ti, VOLTAGE_START, VOLTAGE_END, NUM_SAMPLES, ADC_SAMPLES);
        transmit_sample_data_readable(votlage_samples, NUM_SAMPLES, SAMPLE_VOLTAGE);

        ResistanceSample_TypeDef resistance_samples[NUM_SAMPLES];
        calculate_resistance(votlage_samples, resistance_samples, NUM_SAMPLES);
        transmit_sample_data_readable(resistance_samples, NUM_SAMPLES, SAMPLE_RESISTANCE);

        double conductivity = calculate_conductivity(Au, resistance_samples, NUM_SAMPLES);
        transmit_sample_data_readable(&conductivity, 1, VALUE_CONDUCTIVITY);

        double temperature = measure_temperature();
        transmit_sample_data_readable(&temperature, 1, VALUE_TEMPERATURE);

        double pressure = measure_pressure();
        transmit_sample_data_readable(&pressure, 1, VALUE_PRESSURE);

        HAL_UART_Transmit(&huart6, (uint8_t *)PACKET_END, sizeof(PACKET_END), 1000);

        HAL_Delay(5000);
        // switch (current_task)
        // {
        // case TASK_NONE:
        //     // __WFI();
        //     break;
        // case TASK_MEASURE_TEMPERATURE:
        //     current_task = TASK_NONE;
        //     double temperature = measure_temperature();
        //     HAL_UART_Abort_IT(&huart1);
        //     rs485_transmit((uint8_t *)&temperature, 8);
        //     rs485_receive_IT();
        //     break;
        // case TASK_MEASURE_DEPTH:
        //     current_task = TASK_NONE;
        //     double pressure = measure_pressure();
        //     HAL_UART_Abort_IT(&huart1);
        //     rs485_transmit((uint8_t *)&pressure, 8);
        //     rs485_receive_IT();
        //     break;
        // case TASK_MEASURE_SALINITY:
        //     current_task = TASK_NONE;

        //     VoltageSample_TypeDef voltage_samples[NUM_SAMPLES];
        //     measure_voltage_sweep(voltage_samples, BIDIRECTIONAL, R1_100, Au, 0, 1 << 9, NUM_SAMPLES, 2);

        //     ResistanceSample_TypeDef resistance_samples[NUM_SAMPLES];
        //     calculate_resistance(voltage_samples, resistance_samples, NUM_SAMPLES);

        //     double conductivity = calculate_conductivity(Au, resistance_samples, 10);

        //     double temperature = measure_temperature();
        //     double pressure = measure_pressure();

        //     double salinity = calculate_salinity(conductivity, temperature, pressure);

        //     HAL_UART_Abort_IT(&huart1);
        //     rs485_transmit((uint8_t *)&salinity, 8);
        //     rs485_receive_IT();
        //     break;
        // }
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

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        switch (command)
        {
        case COMMAND_GET_STATUS:
            rs485_transmit((uint8_t *)&status, 1);
            rs485_receive_IT();
            break;
        case COMMAND_GET_TEMPERATURE:
            current_task = TASK_MEASURE_TEMPERATURE;
            rs485_receive_IT();
            break;
        case COMMAND_GET_DEPTH:
            current_task = TASK_MEASURE_DEPTH;
            rs485_receive_IT();
            break;
        case COMMAND_GET_SALINITY:
            current_task = TASK_MEASURE_SALINITY;
            rs485_receive_IT();
            break;
        case COMMAND_RESET:
            NVIC_SystemReset();
        }
    }
}

void rs485_transmit(uint8_t *data, uint16_t size)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    HAL_HalfDuplex_EnableTransmitter(&huart1);
    HAL_UART_Transmit(&huart1, data, size, 1000);
}

void rs485_receive_IT(void)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    HAL_HalfDuplex_EnableReceiver(&huart1);
    HAL_UART_Receive_IT(&huart1, &command, 1);
}

void transmit_sample_data_readable(void *samples, uint16_t num_samples, Sample_Type sample_type)
{
    switch (sample_type)
    {
    case SAMPLE_VOLTAGE:
        VoltageSample_TypeDef *voltage_samples = (VoltageSample_TypeDef *)samples;
        for (uint16_t i = 0; i < num_samples; i++)
        {
            char buf[128];
            sprintf(buf, "DAC Input: %.2f, DAC Output: %.2f, Calib: %.3f, Measurement: %.3f\n", (float)voltage_samples[i].dac_input / 1024 * 3.3, (float)voltage_samples[i].dac_output / 4096 * 3.3, (float)voltage_samples[i].calib / 4096 * 3.3, (float)voltage_samples[i].measurement / 4096 * 3.3);
            HAL_UART_Transmit(&huart6, (uint8_t *)buf, strlen(buf), 1000);
        }
        break;
    case SAMPLE_RESISTANCE:
        ResistanceSample_TypeDef *resistance_samples = (ResistanceSample_TypeDef *)samples;
        for (uint16_t i = 0; i < num_samples; i++)
        {
            char buf[64];
            sprintf(buf, "Voltage: %.2f, Resistance: %.2f\n", (float)resistance_samples[i].voltage / 4096 * 3.3, resistance_samples[i].resistance);
            HAL_UART_Transmit(&huart6, (uint8_t *)buf, strlen(buf), 1000);
        }
        break;
    case VALUE_CONDUCTIVITY:
        double *conductivity_values = (double *)samples;
        for (uint16_t i = 0; i < num_samples; i++)
        {
            char buf[64];
            sprintf(buf, "Conductivity: %.3f\n", conductivity_values[i]);
            HAL_UART_Transmit(&huart6, (uint8_t *)buf, strlen(buf), 1000);
        }
        break;
    case VALUE_TEMPERATURE:
        double *temperature_values = (double *)samples;
        for (uint16_t i = 0; i < num_samples; i++)
        {
            char buf[64];
            sprintf(buf, "Temperature: %.2f\n", temperature_values[i]);
            HAL_UART_Transmit(&huart6, (uint8_t *)buf, strlen(buf), 1000);
        }
        break;
    case VALUE_PRESSURE:
        double *pressure_values = (double *)samples;
        for (uint16_t i = 0; i < num_samples; i++)
        {
            char buf[64];
            sprintf(buf, "Pressure: %.3f\n", pressure_values[i]);
            HAL_UART_Transmit(&huart6, (uint8_t *)buf, strlen(buf), 1000);
        }
        break;
    case VALUE_SALINITY:
        double *salinity_values = (double *)samples;
        for (uint16_t i = 0; i < num_samples; i++)
        {
            char buf[64];
            sprintf(buf, "Salinity: %.3f\n", salinity_values[i]);
            HAL_UART_Transmit(&huart6, (uint8_t *)buf, strlen(buf), 1000);
        }
        break;
    }
}

void transmit_sample_data_binary(void *samples, uint16_t num_samples, Sample_Type sample_type)
{
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
    RS485_Status status = STATUS_ERROR;
    rs485_transmit((uint8_t *)&status, 1);
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
