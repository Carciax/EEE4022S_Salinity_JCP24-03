/* Includes ------------------------------------------------------------------*/
#include "salinity.h"
/* Private variables ---------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

double calculate_salinity(double conductivity, double temperature, double pressure, double standard_conductivity)
{
    double R = conductivity / standard_conductivity;
    double r_t = CONST_C_0 + CONST_C_1 * temperature + CONST_C_2 * temperature * temperature + CONST_C_3 * temperature * temperature * temperature + CONST_C_4 * temperature * temperature * temperature * temperature;
    double R_p = 1 + (CONST_E_1 * pressure + CONST_E_2 * pressure * pressure + CONST_E_3 * pressure * pressure * pressure) / (1 + CONST_D_1 * temperature + CONST_D_2 * temperature * temperature + R * (CONST_D_3 + CONST_D_4 * temperature));
    double R_t = R / (R_p * r_t);
    return CONST_A_0 + CONST_A_1 * pow(R_t, 0.5) + CONST_A_2 * R_t + CONST_A_3 * pow(R_t, 1.5) + CONST_A_4 * R_t * R_t + CONST_A_5 * pow(R_t, 2.5) + (temperature - 15) / (1 + CONST_K * (temperature - 15)) * (CONST_B_0 + CONST_B_1 * pow(R_t, 0.5) + CONST_B_2 * R_t + CONST_B_3 * pow(R_t, 1.5) + CONST_B_4 * R_t * R_t + CONST_B_5 * pow(R_t, 2.5));
}

double calculate_conductivity(Electrode_Type electrode, ResistanceSample_TypeDef *samples, uint16_t num_samples)
{
    switch (electrode)
    {
    case Ti:
        // to be implemented
    case Au:
    case Au_Shielded:
        // average resistance
        double average_resistance = calculate_average_resistance(samples, num_samples);
        // calcualte resistivity, return conductivity
        double resistivity = average_resistance * AU_PAD_AREA / AU_PAD_DISTANCE;
        return 1 / resistivity;
    default:
        break;
    }
    return 0; // should never reach here
}

double calculate_average_resistance(ResistanceSample_TypeDef *samples, uint16_t num_samples)
{
    double sum = 0;
    for (uint16_t i = 0; i < num_samples; i++)
    {
        sum += samples[i].resistance;
    }
    return sum / num_samples;
}

void calculate_resistance(VoltageSample_TypeDef *voltage_samples, ResistanceSample_TypeDef *reistance_samples, uint16_t num_samples)
{
    for (uint16_t i = 0; i < num_samples; i++)
    {
        reistance_samples[i].voltage = voltage_samples[i].dac_output;
        reistance_samples[i].resistance = (double) voltage_samples[i].measurement / voltage_samples[i].calib * CALIBRATION_RESISTANCE;
    }
}

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
    uint8_t status = 0;
    do
    {
        if (HAL_I2C_Mem_Read(&hi2c1, Pressure_device_addr, Pressure_mem_addr_status, 1, &status, 1, 1000) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_Delay(100);
    } while ((status & Pressure_status_finished) == 0);

    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(&hi2c1, Pressure_device_addr, Pressure_mem_addr_temperature, 1, buf, 2, 1000) != HAL_OK)
    {
        Error_Handler();
    }

    uint16_t temperature = (buf[0] << 8) | buf[1];

    return ((double)temperature) * 0.1f;
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
    uint8_t status = 0;
    do
    {
        if (HAL_I2C_Mem_Read(&hi2c1, Pressure_device_addr, Pressure_mem_addr_status, 1, &status, 1, 1000) != HAL_OK)
        {
            Error_Handler();
        }
        HAL_Delay(100);
    } while ((status & Pressure_status_finished) == 0);

    uint8_t buf[4];

    if (HAL_I2C_Mem_Read(&hi2c1, Pressure_device_addr, Pressure_mem_addr_pressure, 1, buf, 4, 1000) != HAL_OK)
    {
        Error_Handler();
    }

    uint32_t pressure = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    
    return ((double) pressure - PASCALS_AT_SEA_LEVEL) * PASCALS_PER_DECIBAR;
}
