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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "voltage.h"
#include "salinity.h"
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

/* USER CODE BEGIN PV */
HAL_StatusTypeDef stat;
uint16_t adc_val;
double salinity;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
double measure_temperature(void);
double measure_pressure(void);
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
    /* USER CODE BEGIN 2 */

    // resistance_init();
    // VoltageSample_TypeDef samples[10];
    // measure_voltage_sweep(samples, BIDIRECTIONAL, R1_100, Ti, 0, 1 << 9, 10, 2);
    // salinity = calculate_salinity(4.0, 15.0, 100.0);

    double temperature = measure_temperature();
    double pressure = measure_pressure();

    HAL_GPIO_WritePin(LED_Green_GPIO_Port, LED_Green_Pin, GPIO_PIN_SET);
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

/** 
 * @brief Measures temperature
 * 
 * @return double: temperature value in degrees Celsius
 */
double measure_temperature(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, Pressure_device_addr, 1, 1000) != HAL_OK)
    {
        Error_Handler();
    }

    // Request temperature
    uint8_t request = Pressure_request_temperature;
    if (HAL_I2C_Mem_Write(&hi2c1, Pressure_device_addr, Pressure_mem_addr_request, 1, &request, 1, 1000) != HAL_OK)
    {
        Error_Handler();
    }

    // Wait for measurement
    uint8_t status;
    do
    {
        if (HAL_I2C_Mem_Read(&hi2c1, Pressure_device_addr, Pressure_mem_addr_status, 1, &status, 1, 1000) != HAL_OK) {
            Error_Handler();
        }
    } while ((status & Pressure_status_finished) == 0);

    uint8_t buf[2];

    if (HAL_I2C_Mem_Read(&hi2c1, Pressure_device_addr, Pressure_mem_addr_pressure, 1, buf, 2, 1000) != HAL_OK) {
        Error_Handler();
    }

    uint16_t temperature = (buf[0] << 8) | buf[1];

    return ((double) temperature) * 0.1f;
}

/**
 * @brief Measures pressure
 * @return double: pressure value in decibars corrected to 0 deibars at 0 meters above sea level
 */
double measure_pressure(void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, Pressure_device_addr, 1, 1000) != HAL_OK)
    {
        Error_Handler();
    }

    // Request pressure
    uint8_t request = Pressure_request_pressure;
    if (HAL_I2C_Mem_Write(&hi2c1, Pressure_device_addr, Pressure_mem_addr_request, 1, &request, 1, 1000) != HAL_OK)
    {
        Error_Handler();
    }

    // Wait for measurement
    uint8_t status;
    do
    {
        if (HAL_I2C_Mem_Read(&hi2c1, Pressure_device_addr, Pressure_mem_addr_status, 1, &status, 1, 1000) != HAL_OK) {
            Error_Handler();
        }
    } while ((status & Pressure_status_finished) == 0);

    uint8_t buf[4];

    if (HAL_I2C_Mem_Read(&hi2c1, Pressure_device_addr, Pressure_mem_addr_pressure, 1, buf, 4, 1000) != HAL_OK) {
        Error_Handler();
    }

    uint32_t pressure = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];

    return ((double) (pressure - PASCALS_AT_SEA_LEVEL)) * PASCALS_PER_DECIBAR;
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
