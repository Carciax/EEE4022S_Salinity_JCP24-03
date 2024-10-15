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
#include "ProbeConfig.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
    DISPLAY_SALINITY_DEPTH = 0x00,
    DISPLAY_TEMPERATURE = 0x01,
    DISPLAY_DEPTH = 0x02,
    DISPLAY_RESISTANCE = 0x03,
    DISPLAY_CONDUCTIVITY = 0x04,
    DISPLAY_SALINITY = 0x05,
    DISPLAY_CONFIG_ELECTRODE = 0x06,
    DISPLAY_CONFIG_R1 = 0x07,
    DISPLAY_CONFIG_VOLTAGE_START = 0x08,
    DISPLAY_CONFIG_VOLTAGE_END = 0x09,
    DISPLAY_CONFIG_VOLTAGE_SAMPLES = 0x0a,
    DISPLAY_CONFIG_ADC_SAMPLES = 0x0b,
    DISPLAY_CONFIG_VOLTAGE_SETTLE_TIME = 0x0c,
    DISPLAY_CONFIG_SEND = 0x0d,
    DISPLAY_CALIB = 0x0e,
    Display_Mode_MAX = 0x0f,
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
    {0xffff0000 | DIGIT_1_MASK | LETTER_R_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_S_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_C_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_N_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_D_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_S_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_A_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_L_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_L_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_E_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_R_MASK, 0xffff0000 | DIGIT_2_MASK | NUMBER_1_MASK, 0xffff0000 | DIGIT_3_MASK | 0},
    {0xffff0000 | DIGIT_1_MASK | LETTER_V_MASK | DECIMAL_POINT, 0xffff0000 | DIGIT_2_MASK | LETTER_S_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_T_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_V_MASK | DECIMAL_POINT, 0xffff0000 | DIGIT_2_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_D_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_V_MASK | DECIMAL_POINT, 0xffff0000 | DIGIT_2_MASK | LETTER_S_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_P_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_S_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_P_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_L_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_S_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_T_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_L_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_C_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_F_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_G_MASK},
    {0xffff0000 | DIGIT_1_MASK | LETTER_C_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_L_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_B_MASK},
};
const uint32_t display_electrode_type[][3] = {
    {0xffff0000 | DIGIT_1_MASK | LETTER_A_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_U_MASK, 0},
    {0xffff0000 | DIGIT_1_MASK | LETTER_T_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_I_MASK, 0},
    {0xffff0000 | DIGIT_1_MASK | LETTER_A_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_U_MASK, 0xffff0000 | DIGIT_3_MASK | LETTER_S_MASK},
};
const uint32_t display_r1_type[][3] = {
    {0xffff0000 | DIGIT_1_MASK | NUMBER_1_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_3_MASK | NUMBER_2_MASK},
    {0xffff0000 | DIGIT_1_MASK | NUMBER_1_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_3_MASK | NUMBER_3_MASK},
    {0xffff0000 | DIGIT_1_MASK | NUMBER_1_MASK, 0xffff0000 | DIGIT_2_MASK | LETTER_E_MASK, 0xffff0000 | DIGIT_3_MASK | NUMBER_4_MASK},
};

uint16_t voltage_map[330];

uint32_t salinity_dma_buff[] = {0xffff0000 | DIGIT_1_MASK | NEGATIVE_SIGN_MASK, 0xffff0000 | DIGIT_2_MASK | NEGATIVE_SIGN_MASK, 0xffff0000 | DIGIT_3_MASK | NEGATIVE_SIGN_MASK};
uint32_t depth_dma_buff[] = {0xffff0000 | DIGIT_1_MASK | NEGATIVE_SIGN_MASK, 0xffff0000 | DIGIT_2_MASK | NEGATIVE_SIGN_MASK, 0xffff0000 | DIGIT_3_MASK | NEGATIVE_SIGN_MASK};

Display_Mode display_mode = DISPLAY_SALINITY_DEPTH;
uint8_t dig1, dig2, dig3;
uint32_t last_tick = 0;

RS485_Expecting expecting_response = EXPECTING_NONE;
uint8_t rx_buffer[256], rx_buffer_size = 0;
uint8_t temperature[DATA_PACKET_SIZE] = {0, 0, 0},
        depth[DATA_PACKET_SIZE] = {0, 0, 0},
        salinity[DATA_PACKET_SIZE] = {0, 0, 0},
        resistance[DATA_PACKET_SIZE] = {0, 0, 0},
        conductivity[DATA_PACKET_SIZE] = {0, 0, 0};
uint16_t v_start = 0, v_end = 329;
ProbeConfig_TypeDef probe_config = {Au, R1_100, BIDIRECTIONAL, 0, 0, 10, 2, STANARD_CONDUCTIVITY, 5};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
uint8_t get_digit(uint8_t digit_number);
int8_t get_change(uint8_t value, uint8_t *prev_val);
void update_buffer_with_voltage(uint32_t *buffer, uint16_t voltage);
void update_buffer_with_int(uint32_t *buffer, uint16_t value);
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
    for (uint16_t i = 0; i < 330; i++)
    {
        voltage_map[i] = (i * 1024) / 330 + 1;
    }
    probe_config.voltage_start = voltage_map[v_start];
    probe_config.voltage_end = voltage_map[v_end];
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
    dig1 = get_digit(1);
    dig2 = get_digit(2);
    dig3 = get_digit(3);

    HAL_DMA_Start(&hdma_tim6_up, (uint32_t)salinity_dma_buff, (uint32_t) & (GPIO_SALINITY->BSRR), 3);
    HAL_TIM_Base_Start(&htim6);
    __HAL_TIM_ENABLE_DMA(&htim6, TIM_DMA_UPDATE);

    HAL_DMA_Start(&hdma_tim17_ch1_up, (uint32_t)depth_dma_buff, (uint32_t) & (GPIO_DEPTH->BSRR), 3);
    HAL_TIM_Base_Start(&htim17);
    __HAL_TIM_ENABLE_DMA(&htim17, TIM_DMA_CC1);
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
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
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
            if ((display_mode == DISPLAY_DEPTH) || (display_mode == DISPLAY_SALINITY_DEPTH))
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
        case EXPECTING_STANARD_CONDUCTIVITY:
            // write std cnd to sd card.
            break;

        case EXPECTING_CONFIG_READY:
            if (rx_buffer[0] == RESPONSE_ACK)
            {
                expecting_response = EXPECTING_ACK;
                rs485_transmit((uint8_t *)&probe_config, CONFIG_PACKET_SIZE);
                rs485_receive_IT(1);
            }
            break;
        case EXPECTING_ACK:
            if (rx_buffer[0] == RESPONSE_ACK)
            {
            }
            break;
        default:
            break;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t command[] = {0}, data[] = {0, 0, 0};
    int8_t change;
    switch (GPIO_Pin)
    {
    case DIG1_1_Pin:
        uint8_t dig = get_digit(1);
        change = get_change(dig, &dig1);
        display_mode = (display_mode + change + Display_Mode_MAX) % Display_Mode_MAX;

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
        case DISPLAY_CONFIG_ELECTRODE:
            depth_dma_buff[0] = display_electrode_type[probe_config.electrode][0];
            depth_dma_buff[1] = display_electrode_type[probe_config.electrode][1];
            depth_dma_buff[2] = display_electrode_type[probe_config.electrode][2];
            break;
        case DISPLAY_CONFIG_R1:
            depth_dma_buff[0] = display_r1_type[probe_config.r1][0];
            depth_dma_buff[1] = display_r1_type[probe_config.r1][1];
            depth_dma_buff[2] = display_r1_type[probe_config.r1][2];
            break;
        case DISPLAY_CONFIG_VOLTAGE_START:
            update_buffer_with_voltage(depth_dma_buff, probe_config.voltage_start);
            break;
        case DISPLAY_CONFIG_VOLTAGE_END:
            update_buffer_with_voltage(depth_dma_buff, probe_config.voltage_end);
            break;
        case DISPLAY_CONFIG_VOLTAGE_SAMPLES:
            data[0] = probe_config.num_samples / 100;
            data[1] = (probe_config.num_samples % 100) / 10;
            data[2] = probe_config.num_samples % 10;
            update_buffer(depth_dma_buff, data);
            break;
        case DISPLAY_CONFIG_ADC_SAMPLES:
            data[0] = probe_config.adc_samples / 100;
            data[1] = (probe_config.adc_samples % 100) / 10;
            data[2] = probe_config.adc_samples % 10;
            update_buffer(depth_dma_buff, data);
            break;
        case DISPLAY_CONFIG_SEND:
        case DISPLAY_CALIB:
            depth_dma_buff[0] = 0xffff0000 | DIGIT_1_MASK | NEGATIVE_SIGN_MASK;
            depth_dma_buff[1] = 0xffff0000 | DIGIT_2_MASK | NEGATIVE_SIGN_MASK;
            depth_dma_buff[2] = 0xffff0000 | DIGIT_3_MASK | NEGATIVE_SIGN_MASK;
            break;
        default:
            break;
        }
        break;
    case DIG2_1_Pin:
        change = get_change(get_digit(2), &dig2);
        switch (display_mode)
        {
        case DISPLAY_CONFIG_ELECTRODE:
            probe_config.electrode = (probe_config.electrode + change + Electrode_Type_MAX) % Electrode_Type_MAX;
            depth_dma_buff[0] = display_electrode_type[probe_config.electrode][0];
            depth_dma_buff[1] = display_electrode_type[probe_config.electrode][1];
            depth_dma_buff[2] = display_electrode_type[probe_config.electrode][2];
            break;
        case DISPLAY_CONFIG_R1:
            probe_config.r1 = (probe_config.r1 + change + R1_Type_MAX) % R1_Type_MAX;
            depth_dma_buff[0] = display_r1_type[probe_config.r1][0];
            depth_dma_buff[1] = display_r1_type[probe_config.r1][1];
            depth_dma_buff[2] = display_r1_type[probe_config.r1][2];
            break;
        case DISPLAY_CONFIG_VOLTAGE_START:
            v_start = (v_start + change * 10 + 330) % 330;
            probe_config.voltage_start = voltage_map[v_start];
            update_buffer_with_voltage(depth_dma_buff, probe_config.voltage_start);
            break;
        case DISPLAY_CONFIG_VOLTAGE_END:
            v_end = (v_end + change * 10 + 330) % 330;
            probe_config.voltage_end = voltage_map[v_end];
            update_buffer_with_voltage(depth_dma_buff, probe_config.voltage_end);
            break;
        case DISPLAY_CONFIG_VOLTAGE_SAMPLES:
            probe_config.num_samples = (probe_config.num_samples + change * 10) % 1000;
            update_buffer_with_int(depth_dma_buff, probe_config.num_samples);
            break;
        case DISPLAY_CONFIG_ADC_SAMPLES:
            probe_config.adc_samples = (probe_config.adc_samples + change * 10) % 1000;
            update_buffer_with_int(depth_dma_buff, probe_config.adc_samples);
            break;
        case DISPLAY_CONFIG_VOLTAGE_SETTLE_TIME:
            probe_config.voltage_settle_time = (probe_config.voltage_settle_time + change * 10) % 1000;
            update_buffer_with_int(depth_dma_buff, probe_config.voltage_settle_time);
        default:
            break;
        }
        break;
    case DIG3_1_Pin:
        change = get_change(get_digit(3), &dig3);
        switch (display_mode)
        {
        case DISPLAY_CONFIG_VOLTAGE_START:
            v_start = (v_start + change + 330) % 330;
            probe_config.voltage_start = voltage_map[v_start];
            update_buffer_with_voltage(depth_dma_buff, probe_config.voltage_start);
            break;
        case DISPLAY_CONFIG_VOLTAGE_END:
            v_end = (v_end + change + 330) % 330;
            probe_config.voltage_end = voltage_map[v_end];
            update_buffer_with_voltage(depth_dma_buff, probe_config.voltage_end);
            break;
        case DISPLAY_CONFIG_VOLTAGE_SAMPLES:
            probe_config.num_samples = (probe_config.num_samples + change) % 1000;
            update_buffer_with_int(depth_dma_buff, probe_config.num_samples);
            break;
        case DISPLAY_CONFIG_ADC_SAMPLES:
            probe_config.adc_samples = (probe_config.adc_samples + change) % 1000;
            update_buffer_with_int(depth_dma_buff, probe_config.adc_samples);
            break;
        case DISPLAY_CONFIG_VOLTAGE_SETTLE_TIME:
            probe_config.voltage_settle_time = (probe_config.voltage_settle_time + change) % 1000;
            update_buffer_with_int(depth_dma_buff, probe_config.voltage_settle_time);
        default:
            break;
        }
        break;
    case SW1_Pin:
        switch (display_mode)
        {
        case DISPLAY_SALINITY_DEPTH:
            expecting_response = EXPECTING_SALINITY_DEPTH;
            command[0] = COMMAND_GET_SALINITY;
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

        case DISPLAY_CONFIG_ELECTRODE:
        case DISPLAY_CONFIG_R1:
        case DISPLAY_CONFIG_VOLTAGE_START:
        case DISPLAY_CONFIG_VOLTAGE_END:
        case DISPLAY_CONFIG_VOLTAGE_SAMPLES:
        case DISPLAY_CONFIG_ADC_SAMPLES:
        case DISPLAY_CONFIG_SEND:
            expecting_response = EXPECTING_CONFIG_READY;
            command[0] = COMMAND_SET_CONFIG;
            rs485_transmit(command, 1);
            rs485_receive_IT(1);
            break;
        case DISPLAY_CALIB:
            expecting_response = EXPECTING_STANARD_CONDUCTIVITY;
            command[0] = COMMNAD_GET_STANDARD_CONDUCTIVITY;
            rs485_transmit(command, 1);
            rs485_receive_IT(sizeof(uint32_t));
            break;
        default:
            break;
        }
        break;
    case SW2_Pin:
        command[0] = COMMAND_RESET;
        rs485_transmit(command, 1);
        HAL_NVIC_SystemReset();
        break;
    default:
        break;
    }
}

uint8_t get_digit(uint8_t digit_number)
{
    HAL_Delay(10);
    switch (digit_number)
    {
    case 1:
        return (HAL_GPIO_ReadPin(DIG1_8_GPIO_Port, DIG1_8_Pin) << 3) |
               (HAL_GPIO_ReadPin(DIG1_4_GPIO_Port, DIG1_4_Pin) << 2) |
               (HAL_GPIO_ReadPin(DIG1_2_GPIO_Port, DIG1_2_Pin) << 1) |
               HAL_GPIO_ReadPin(DIG1_1_GPIO_Port, DIG1_1_Pin);
    case 2:
        return (HAL_GPIO_ReadPin(DIG2_8_GPIO_Port, DIG2_8_Pin) << 3) |
               (HAL_GPIO_ReadPin(DIG2_4_GPIO_Port, DIG2_4_Pin) << 2) |
               (HAL_GPIO_ReadPin(DIG2_2_GPIO_Port, DIG2_2_Pin) << 1) |
               HAL_GPIO_ReadPin(DIG2_1_GPIO_Port, DIG2_1_Pin);
    case 3:
        return (HAL_GPIO_ReadPin(DIG3_8_GPIO_Port, DIG3_8_Pin) << 3) |
               (HAL_GPIO_ReadPin(DIG3_4_GPIO_Port, DIG3_4_Pin) << 2) |
               (HAL_GPIO_ReadPin(DIG3_2_GPIO_Port, DIG3_2_Pin) << 1) |
               HAL_GPIO_ReadPin(DIG3_1_GPIO_Port, DIG3_1_Pin);
    default:
        return 0;
    }
}

int8_t get_change(uint8_t value, uint8_t *prev_val)
{
    int8_t change;
    if ((value + 1) % 10 == *prev_val)
    {
        change = -1;
    }
    else if ((value + 9) % 10 == *prev_val)
    {
        change = +1;
    }
    else
    {
        change = 0;
    }
    *prev_val = value;
    return change;
}

void update_buffer_with_voltage(uint32_t *buffer, uint16_t voltage)
{
    // voltage is 10 bit
    uint8_t data[3] = {0, 0, 0};
    voltage = (voltage * 330) / 1024;
    data[0] = voltage / 100 | DECIMAL_POINT;
    data[1] = (voltage % 100) / 10;
    data[2] = voltage % 10;
    update_buffer(buffer, data);
}

void update_buffer_with_int(uint32_t *buffer, uint16_t value)
{
    uint8_t data[3] = {0, 0, 0};
    data[0] = value / 100;
    data[1] = (value % 100) / 10;
    data[2] = value % 10;
    update_buffer(buffer, data);
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
        if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET)
        {
            HAL_NVIC_SystemReset();
        }
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
