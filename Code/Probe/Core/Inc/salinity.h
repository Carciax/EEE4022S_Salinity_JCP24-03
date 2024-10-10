#ifndef __SALINITY_H
#define __SALINITY_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "voltage.h"
#include "math.h"
/* Private defines -----------------------------------------------------------*/
#define CONST_K 0.0162

#define CONST_A_0 0.0080
#define CONST_A_1 -0.1692
#define CONST_A_2 25.3851
#define CONST_A_3 14.0941
#define CONST_A_4 -7.0261
#define CONST_A_5 2.7081

#define CONST_B_0 0.0005
#define CONST_B_1 -0.0056
#define CONST_B_2 -0.0066
#define CONST_B_3 -0.0375
#define CONST_B_4 0.0636
#define CONST_B_5 -0.0144

#define CONST_C_0 0.6766097
#define CONST_C_1 0.0200564
#define CONST_C_2 0.0001104259
#define CONST_C_3 -0.00000069698
#define CONST_C_4 0.0000000010031

#define CONST_D_1 0.03426
#define CONST_D_2 0.0004464
#define CONST_D_3 -0.004215
#define CONST_D_4 -0.003107

#define CONST_E_1 0.00002070
#define CONST_E_2 -0.000000006370
#define CONST_E_3 0.000000000000003989

#define AU_PAD_AREA 0.0004
#define AU_PAD_DISTANCE 0.01

#define CALIBRATION_RESISTANCE 5
#define CORR_EQN_P1 87.3433
#define CORR_EQN_P2 92.3206
#define CORR_EQN_Q1 91.8310

#define PASCALS_AT_SEA_LEVEL 101325
#define PASCALS_PER_DECIBAR 0.0001

/* Private typedef -----------------------------------------------------------*/

typedef struct
{
    uint16_t voltage;
    double resistance;
} ResistanceSample_TypeDef;

/* Functions prototypes ---------------------------------------------*/
double calculate_salinity(double conductivity, double temperature, double pressure, double standard_conductivity);
double calculate_conductivity(Electrode_Type electrode, ResistanceSample_TypeDef *samples, uint16_t num_samples);
void calculate_resistance(VoltageSample_TypeDef *voltage_samples, ResistanceSample_TypeDef *reistance_samples, uint16_t num_samples);
double calculate_average_resistance(ResistanceSample_TypeDef *samples, uint16_t num_samples);
double measure_temperature(void);
double measure_pressure(void);

#endif // __SALINITY_H