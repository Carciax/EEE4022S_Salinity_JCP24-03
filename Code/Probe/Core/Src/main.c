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
#include "ProbeConfig.h"
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
    TASK_MEASURE_RESISTANCE = 0x03,
    TASK_MEASURE_CONDUCTIVITY = 0x04,
    TASK_MEASURE_SALINITY = 0x05,
    TASK_WAITING_FOR_CONFIG = 0x06,
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
uint8_t rx_buffer[CONFIG_PACKET_SIZE];
RS485_Status status = STATUS_IDLE;
ProbeConfig_TypeDef probe_config = {Au, R1_100, BIDIRECTIONAL, 93, 868, 10, 2, STANARD_CONDUCTIVITY, 12};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void rs485_transmit(uint8_t *data, uint16_t size);
void rs485_transmit_double(double value);
void rs485_receive_IT(uint16_t size);
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

    /* Initialize interrupts */
    MX_NVIC_Init();
    /* USER CODE BEGIN 2 */
    voltage_init();
    HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    double pressure, temperature, resistance, conductivity, salinity;
    VoltageSample_TypeDef voltage_samples[MAX_SAMPLES];
    ResistanceSample_TypeDef resistance_samples[MAX_SAMPLES];
    memset(voltage_samples, 0, sizeof(voltage_samples));
    HAL_Delay(5000);

    // GPIOSW->BSRR = SW_Ti1_GND_Pin | SW_Ti2_GND_Pin;
    // GPIOSW->BSRR = SW_Ti_Pin_Msk << 16;
    // for (uint16_t i = 0; i < 20; i++)
    // {
    //     measure_ac_sweep(
    //         voltage_samples,
    //         R1_10k,
    //         Ti,
    //         4,
    //         256,
    //         50);
    // }
    // transmit_sample_data_binary(voltage_samples, 256, SAMPLE_VOLTAGE);

    // measure_voltage_sweep(
    //     voltage_samples,
    //     BIDIRECTIONAL,
    //     R1_100,
    //     Au_Shielded,
    //     V_MIN,
    //     V_MAX,
    //     5,
    //     5,
    //     10,
    //     0);
    // HAL_Delay(2000);

    measure_voltage_sweep(
        voltage_samples,
        BIDIRECTIONAL,
        R1_100,
        Au_Shielded,
        512,
        512,
        1,
        5,
        100,
        0);
    transmit_sample_data_binary(voltage_samples, 1, SAMPLE_VOLTAGE);
    temperature = measure_temperature();
    transmit_sample_data_readable(&temperature, 1, VALUE_TEMPERATURE);

    rs485_receive_IT(1);

    while (1)
    {

        // HAL_UART_Transmit(&huart6, (uint8_t *)PACKET_START, sizeof(PACKET_START), 1000);

        // VoltageSample_TypeDef votlage_samples[NUM_SAMPLES];
        // measure_voltage_sweep(votlage_samples, BIDIRECTIONAL, R1_100, Ti, VOLTAGE_START, VOLTAGE_END, NUM_SAMPLES, ADC_SAMPLES);
        // transmit_sample_data_readable(votlage_samples, NUM_SAMPLES, SAMPLE_VOLTAGE);

        // ResistanceSample_TypeDef resistance_samples[NUM_SAMPLES];
        // calculate_resistance(votlage_samples, resistance_samples, NUM_SAMPLES);
        // transmit_sample_data_readable(resistance_samples, NUM_SAMPLES, SAMPLE_RESISTANCE);

        // double conductivity = calculate_conductivity(Au_Shielded, resistance_samples, NUM_SAMPLES);
        // transmit_sample_data_readable(&conductivity, 1, VALUE_CONDUCTIVITY);

        // double temperature = measure_temperature();
        // transmit_sample_data_readable(&temperature, 1, VALUE_TEMPERATURE);

        // double pressure = measure_pressure();
        // transmit_sample_data_readable(&pressure, 1, VALUE_PRESSURE);

        // double salinity = calculate_salinity(conductivity, temperature, pressure);
        // transmit_sample_data_readable(&salinity, 1, VALUE_SALINITY);

        // HAL_UART_Transmit(&huart6, (uint8_t *)PACKET_END, sizeof(PACKET_END), 1000);

        // HAL_Delay(5000);

        switch (current_task)
        {
        case TASK_NONE:
            // __WFI();
            break;
        case TASK_MEASURE_TEMPERATURE:
            status = STATUS_BUSY;
            current_task = TASK_NONE;

            temperature = measure_temperature();
            HAL_UART_Abort_IT(&huart1);
            rs485_transmit_double(temperature);

            status = STATUS_IDLE;
            rs485_receive_IT(1);
            break;
        case TASK_MEASURE_DEPTH:
            status = STATUS_BUSY;
            current_task = TASK_NONE;

            pressure = measure_pressure();
            HAL_UART_Abort_IT(&huart1);
            rs485_transmit_double(pressure);

            status = STATUS_IDLE;
            rs485_receive_IT(1);
            break;
        case TASK_MEASURE_RESISTANCE:
            status = STATUS_BUSY;
            current_task = TASK_NONE;

            measure_voltage_sweep(
                voltage_samples,
                probe_config.direction,
                probe_config.r1,
                probe_config.electrode,
                probe_config.voltage_start,
                probe_config.voltage_end,
                probe_config.num_samples,
                probe_config.adc_samples,
                probe_config.voltage_settle_time,
                0);
            calculate_resistance(voltage_samples, resistance_samples, probe_config.num_samples);

            resistance = calculate_average_resistance(resistance_samples, probe_config.num_samples);

            HAL_UART_Abort_IT(&huart1);
            rs485_transmit_double(resistance);

            status = STATUS_IDLE;
            rs485_receive_IT(1);
            break;
        case TASK_MEASURE_CONDUCTIVITY:
            status = STATUS_BUSY;
            current_task = TASK_NONE;

            measure_voltage_sweep(
                voltage_samples,
                probe_config.direction,
                probe_config.r1,
                probe_config.electrode,
                probe_config.voltage_start,
                probe_config.voltage_end,
                probe_config.num_samples,
                probe_config.adc_samples,
                probe_config.voltage_settle_time,
                0);
            calculate_resistance(voltage_samples, resistance_samples, probe_config.num_samples);

            conductivity = calculate_conductivity(probe_config.electrode, resistance_samples, probe_config.num_samples);

            HAL_UART_Abort_IT(&huart1);
            rs485_transmit_double(conductivity);

            status = STATUS_IDLE;
            rs485_receive_IT(1);
            break;
        case TASK_MEASURE_SALINITY:
            status = STATUS_BUSY;
            current_task = TASK_NONE;

            measure_voltage_sweep(
                voltage_samples,
                probe_config.direction,
                probe_config.r1,
                probe_config.electrode,
                probe_config.voltage_start,
                probe_config.voltage_end,
                probe_config.num_samples,
                probe_config.adc_samples,
                probe_config.voltage_settle_time,
                0);
            calculate_resistance(voltage_samples, resistance_samples, probe_config.num_samples);

            conductivity = calculate_conductivity(probe_config.electrode, resistance_samples, probe_config.num_samples);
            temperature = measure_temperature();
            pressure = measure_pressure();

            salinity = calculate_salinity(conductivity, temperature, pressure, (double)probe_config.stardard_conductivity / 1e6);
            HAL_UART_Abort_IT(&huart1);
            rs485_transmit_double(salinity);

            status = STATUS_IDLE;
            rs485_receive_IT(1);
            break;
        case TASK_WAITING_FOR_CONFIG:
            break;
        default:
            break;
        }

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

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void)
{
    /* USART1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    /* USART6_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_GPIO_TogglePin(LED_Green_GPIO_Port, LED_Green_Pin);
    uint8_t response;
    if (huart->Instance == USART1)
    {
        if (current_task == TASK_WAITING_FOR_CONFIG)
        {
            current_task = TASK_NONE;
            response = RESPONSE_ACK;
            rs485_transmit(&response, 1);

            memcpy(&probe_config, rx_buffer, sizeof(ProbeConfig_TypeDef));

            rs485_receive_IT(1);
        }
        else
        {
            switch ((RS485_Command)rx_buffer[0])
            {
            case COMMAND_GET_STATUS:
                rs485_transmit((uint8_t *)&status, 1);
                rs485_receive_IT(1);
                break;
            case COMMAND_GET_TEMPERATURE:
                current_task = TASK_MEASURE_TEMPERATURE;
                rs485_receive_IT(1);
                break;
            case COMMAND_GET_DEPTH:
                current_task = TASK_MEASURE_DEPTH;
                rs485_receive_IT(1);
                break;
            case COMMAND_GET_RESISTANCE:
                current_task = TASK_MEASURE_RESISTANCE;
                rs485_receive_IT(1);
                break;
            case COMMAND_GET_CONDUCTIVITY:
                current_task = TASK_MEASURE_CONDUCTIVITY;
                rs485_receive_IT(1);
                break;
            case COMMAND_GET_SALINITY:
                current_task = TASK_MEASURE_SALINITY;
                rs485_receive_IT(1);
                break;
            case COMMAND_SET_CONFIG:
                current_task = TASK_WAITING_FOR_CONFIG;
                response = RESPONSE_ACK;
                rs485_transmit(&response, 1);
                rs485_receive_IT(CONFIG_PACKET_SIZE);
                break;
            case COMMAND_RESET:
                HAL_NVIC_SystemReset();
                break;
            default:
                response = RESPONSE_NACK;
                rs485_transmit(&response, 1);
                rs485_receive_IT(1);
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

void rs485_transmit_double(double value)
{
    if ((value >= 1000) | (value <= -100))
    {
        uint8_t digits[3] = {NEGATIVE_SIGN, NEGATIVE_SIGN, NEGATIVE_SIGN};
        return rs485_transmit(digits, DATA_PACKET_SIZE);
    }

    uint8_t digits[3];

    if (value == 0)
    {
        digits[0] = DECIMAL_POINT;
        digits[1] = 0;
        digits[2] = 0;
    }
    else if (value > 0)
    {
        int8_t log_value = (int8_t)log10(value);
        if (log_value > 0)
        {
            value = value * pow(10, -log_value);
        }

        digits[0] = (uint8_t)value;
        value = fmod(value, 1) * 10;
        digits[1] = (uint8_t)value;
        value = fmod(value, 1) * 10;
        digits[2] = (uint8_t)round(value);

        if (log_value > 0)
        {
            if (log_value < 2)
            {
                digits[log_value] |= DECIMAL_POINT;
            }
        }
        else
        {

            digits[0] |= DECIMAL_POINT;
        }
    }
    else
    {
        value = -value;
        int8_t log_value = (int8_t)log10(value) + 1;
        if (log_value > 0)
        {
            value = value * pow(10, -log_value);
        }
        else if (log_value == 0)
        {
            value = value / 10;
        }

        digits[0] = NEGATIVE_SIGN;
        value = fmod(value, 1) * 10;
        digits[1] = (uint8_t)value;
        value = fmod(value, 1) * 10;
        digits[2] = (uint8_t)round(value);

        if (log_value > 0)
        {
            if (log_value < 2)
            {
                digits[log_value] |= DECIMAL_POINT;
            }
        }
        else
        {
            digits[1] |= DECIMAL_POINT;
        }
    }

    return rs485_transmit(digits, DATA_PACKET_SIZE);
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

void transmit_sample_data_readable(void *samples, uint16_t num_samples, Sample_Type sample_type)
{
    switch (sample_type)
    {
    case SAMPLE_VOLTAGE:
        VoltageSample_TypeDef *voltage_samples = (VoltageSample_TypeDef *)samples;
        for (uint16_t i = 0; i < num_samples; i++)
        {
            char buf[128];
            sprintf(buf, "DAC Input: %.2f, DAC Output: %.2f, Calib x 11: %.3f, Measurement x 11: %.3f\n", (float)voltage_samples[i].dac_input / 1024 * 3.3, (float)voltage_samples[i].dac_output / 4096 * 3.3, (float)voltage_samples[i].calib / 4096 * 3.3, (float)voltage_samples[i].measurement / 4096 * 3.3);
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
            sprintf(buf, "\n\nTemperature: %.1f", temperature_values[i]);
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
    switch (sample_type)
    {
    case SAMPLE_VOLTAGE:
        VoltageSample_TypeDef *voltage_samples = (VoltageSample_TypeDef *)samples;
        char buf[] = "\n\n|DAC Input|DAC Output|Calib x 11|Measurement x 11|\n";
        HAL_UART_Transmit(&huart6, (uint8_t *)buf, strlen(buf), 1000);
        for (uint16_t i = 0; i < num_samples; i++)
        {
            sprintf(buf, "%d,%d,%d,%d;", voltage_samples[i].dac_input, voltage_samples[i].dac_output, voltage_samples[i].calib, voltage_samples[i].measurement);
            HAL_UART_Transmit(&huart6, (uint8_t *)buf, strlen(buf), 1000);
        }
        break;
    case SAMPLE_RESISTANCE:
        // ResistanceSample_TypeDef *resistance_samples = (ResistanceSample_TypeDef *)samples;
        break;
    case VALUE_CONDUCTIVITY:
        // double *conductivity_values = (double *)samples;
        break;
    case VALUE_TEMPERATURE:
        // double *temperature_values = (double *)samples;
        break;
    case VALUE_PRESSURE:
        // double *pressure_values = (double *)samples;
        break;
    case VALUE_SALINITY:
        // double *salinity_values = (double *)samples;
        break;
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
    status = STATUS_ERROR;
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
