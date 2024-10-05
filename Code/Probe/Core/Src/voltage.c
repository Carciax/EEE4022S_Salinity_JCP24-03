/* Includes ------------------------------------------------------------------*/
#include "voltage.h"
/* Private variables ---------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
void voltage_init (void)
{
    dac_init();
    adc_set_channel(ADC_CHANNEL_SIGNAL);
    // adc_set_channel(ADC_CHANNEL_AMP);
    reset_muxs();
    HAL_TIM_Base_Start(&htim10);
}

void us_delay(uint16_t micro_s)
{
    //Requires a timer running at 1Mhz
	__HAL_TIM_SET_COUNTER(&htim10,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim10) < micro_s);  // wait for the counter to reach the us input in the parameter
}

void dac_init (void)
{
    if (HAL_I2C_IsDeviceReady(&hi2c1, DAC_device_addr, 2, 1000) != HAL_OK)
    {
        Error_Handler();
    }
    // Reset the DAC
    dac_write(DAC_mem_addr_trigger, DAC_trigger_reset | DAC_trigger_config_reset);
    // Power up DAC
    dac_write(DAC_mem_addr_general_config, DAC_general_config_pdn_power_up);
}

void dac_drain (void)
{
    dac_write(DAC_mem_addr_dac_data, 0x0000);
    dac_write(DAC_mem_addr_general_config, DAC_general_config_pdn_power_down_10k);
    HAL_Delay(10);
    dac_write(DAC_mem_addr_general_config, DAC_general_config_pdn_power_up);
}

void dac_set_voltage (uint16_t voltage)
{
    dac_write(DAC_mem_addr_dac_data, voltage << 2);
}

void dac_write (uint16_t memory_addr, uint16_t data)
{
    uint8_t buf[2] = {0};
    buf[0] = data >> 8;
    buf[1] = data & 0xff;
    if (HAL_I2C_Mem_Write(&hi2c1, DAC_device_addr, memory_addr, 1, buf, 2, 1000) != HAL_OK)
    {
        Error_Handler();
    }
}

void adc_set_channel (ADC_Channel channel)
{
    ADC_ChannelConfTypeDef sConfig = {channel, 1, ADC_SAMPLETIME_3CYCLES, 0};
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

uint16_t adc_average (uint16_t adc_samples)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < adc_samples; i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1000);
        sum += HAL_ADC_GetValue(&hadc1);
    }
    return (uint16_t)sum / adc_samples;
}

void reset_muxs (void)
{
    GPIOSW->BSRR = 0xffff << 16;
}

uint16_t measure_dac_voltage (uint16_t adc_samples)
{
    GPIOSW->BSRR = SW_R1_Calib_Pin;
    us_delay(VOLTAGE_SETTLE_TIME);
    uint16_t adc_val = adc_average(adc_samples);
    GPIOSW->BSRR = SW_R1_Calib_Pin << 16;
    return adc_val;
}

uint16_t measure_calib_voltage (uint16_t adc_samples) {
    GPIOSW->BSRR = SW_R1_Calib_Pin;
    us_delay(VOLTAGE_SETTLE_TIME);
    uint16_t adc_val = adc_average(adc_samples);
    GPIOSW->BSRR = SW_R1_Calib_Pin << 16;
    return adc_val;
}

uint16_t measure_pin_voltage (Direction direction, Electrode_Type electrode, uint16_t adc_samples)
{
    uint16_t adc_forward_val = 0, adc_reverse_val = 0;

    switch (direction)
    {
    case BIDIRECTIONAL:
        switch (electrode)
        {
        case Au:
            GPIOSW->BSRR = SW_R1_Au2_Pin | SW_Au1_GND_Pin;
            us_delay(VOLTAGE_SETTLE_TIME);
            adc_reverse_val = adc_average(adc_samples);
            GPIOSW->BSRR = (SW_R1_Au2_Pin | SW_Au1_GND_Pin) << 16;
            break;
        case Ti:
            GPIOSW->BSRR = SW_R1_Ti2_Pin | SW_Ti1_GND_Pin;
            us_delay(VOLTAGE_SETTLE_TIME);
            adc_reverse_val = adc_average(adc_samples);
            GPIOSW->BSRR = (SW_R1_Ti2_Pin | SW_Ti1_GND_Pin) << 16;
            break;
        case Au_Shielded:
            GPIOSW->BSRR = SW_R1_Au2_Pin | SW_Au1_GND_Pin | SW_Shield2_V_Pin | SW_Shield1_GND_Pin;
            us_delay(VOLTAGE_SETTLE_TIME);
            adc_reverse_val = adc_average(adc_samples);
            GPIOSW->BSRR = (SW_R1_Au2_Pin | SW_Au1_GND_Pin | SW_Shield2_V_Pin | SW_Shield1_GND_Pin) << 16;
            break;
        }
    case UNIDIRECTIONAL:
        switch (electrode)
        {
        case Au:
            GPIOSW->BSRR = SW_R1_Au1_Pin | SW_Au2_GND_Pin;
            us_delay(VOLTAGE_SETTLE_TIME);
            adc_forward_val = adc_average(adc_samples);
            GPIOSW->BSRR = (SW_R1_Au1_Pin | SW_Au2_GND_Pin) << 16;
            break;
        case Ti:
            GPIOSW->BSRR = SW_R1_Ti1_Pin | SW_Ti2_GND_Pin;
            us_delay(VOLTAGE_SETTLE_TIME);
            adc_forward_val = adc_average(adc_samples);
            GPIOSW->BSRR = (SW_R1_Ti1_Pin | SW_Ti2_GND_Pin) << 16;
            break;
        case Au_Shielded:
            GPIOSW->BSRR = SW_R1_Au1_Pin | SW_Au2_GND_Pin | SW_Shield1_V_Pin | SW_Shield2_GND_Pin;
            us_delay(VOLTAGE_SETTLE_TIME);
            adc_forward_val = adc_average(adc_samples);
            GPIOSW->BSRR = (SW_R1_Au1_Pin | SW_Au2_GND_Pin | SW_Shield1_V_Pin | SW_Shield2_GND_Pin) << 16;
            break;
        }
    }

    if (direction == BIDIRECTIONAL) {
        return (adc_forward_val + adc_reverse_val) / 2;
    }
    return adc_forward_val;
}

void measure_voltage_sweep (VoltageSample_TypeDef* samples, Direction direction, R1_Type r1, Electrode_Type electrode, uint16_t dac_start, uint16_t dac_stop, uint16_t num_samples, uint16_t adc_samples)
{
    uint16_t dac_voltage = dac_start;
    uint16_t dac_step = (dac_stop - dac_start) / num_samples;

    reset_muxs();
    dac_drain();

    switch (r1)
    {
    case R1_100:
        GPIOSW->BSRR = SW_R1_100_Pin;
        break;
    case R1_1k:
        GPIOSW->BSRR = SW_R1_1k_Pin;
        break;
    case R1_10k:
        GPIOSW->BSRR = SW_R1_10k_Pin;
        break;
    }

    for (uint16_t i = 0; i < num_samples; i++) {
        dac_set_voltage(dac_voltage);

        samples[i].dac_input = dac_voltage;

        adc_set_channel(ADC_CHANNEL_DAC);
        samples[i].dac_output = measure_dac_voltage(adc_samples);

        adc_set_channel(ADC_CHANNEL_SIGNAL);
        samples[i].calib = measure_calib_voltage(adc_samples);

        samples[i].measurement = measure_pin_voltage(direction, electrode, adc_samples);

        dac_voltage += dac_step;
    }

    reset_muxs();
    dac_drain();
}