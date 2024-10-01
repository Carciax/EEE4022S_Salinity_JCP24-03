/* Includes ------------------------------------------------------------------*/
#include "salinity.h"

/* Private variables ---------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

double calculate_salinity(double conductivity, double temperature, double pressure)
{
    double R = conductivity / STANARD_CONDUCTIVITY;
    double r_t = CONST_C_0 + CONST_C_1 * temperature + CONST_C_2 * temperature * temperature + CONST_C_3 * temperature * temperature * temperature + CONST_C_4 * temperature * temperature * temperature * temperature;
    double R_p = 1 + (CONST_E_1 * pressure + CONST_E_2 * pressure * pressure + CONST_E_3 * pressure * pressure * pressure) / (1 + CONST_D_1 * temperature + CONST_D_2 * temperature * temperature + R * (CONST_D_3 + CONST_D_4 * temperature));
    double R_t = R / (R_p * r_t);
    return CONST_A_0 + CONST_A_1 * pow(R_t, 0.5) + CONST_A_2 * R_t + CONST_A_3 * pow(R_t, 1.5) + CONST_A_4 * R_t * R_t + CONST_A_5 * pow(R_t, 2.5) + (temperature - 15) / (1 + CONST_K * (temperature - 15)) * (CONST_B_0 + CONST_B_1 * pow(R_t, 0.5) + CONST_B_2 * R_t + CONST_B_3 * pow(R_t, 1.5) + CONST_B_4 * R_t * R_t + CONST_B_5 * pow(R_t, 2.5));
}

double calculate_conductivity(Electrode_Type electrode, ResistanceSample_TypeDef *samples, uint16_t num_samples)
{
    switch (electrode)
    {
    case Au:
    {
        // average resistance
        double sum = 0;
        for (uint16_t i = 0; i < num_samples; i++)
        {
            sum += samples[i].resistance;
        }
        double average_resistance = sum / num_samples;
        // calcualte resistivity, return conductivity
        double resistivity = average_resistance * AU_PAD_AREA / AU_PAD_DISTANCE;
        return 1 / resistivity;
    }
    case Ti:
    {
        return 0;
    }
    }
}

void calculate_resistance(VoltageSample_TypeDef *voltage_samples, ResistanceSample_TypeDef *reistance_samples, uint16_t num_samples)
{
    for (uint16_t i = 0; i < num_samples; i++)
    {
        reistance_samples[i].voltage = voltage_samples[i].dac_output;
        reistance_samples[i].resistance = voltage_samples[i].measurement / voltage_samples[i].calib * CALIBRATION_RESISTANCE;
    }
}
