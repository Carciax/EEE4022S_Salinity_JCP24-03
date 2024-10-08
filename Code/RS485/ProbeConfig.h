#ifndef __PROBECONFIGURATION_H
#define __PROBECONFIGURATION_H

#include "main.h"

typedef enum {
    Au = 0U,
    Ti = 1U,
    Au_Shielded = 2U,
    Electrode_Type_MAX = 3U
} Electrode_Type;

typedef enum {
    R1_100 = 0U,
    R1_1k = 1U,
    R1_10k = 2U,
    R1_Type_MAX = 3U
} R1_Type;

typedef enum {
    UNIDIRECTIONAL = 0U,
    BIDIRECTIONAL = 1U,
    Direction_MAX = 2U
} Direction;

typedef struct {
    Electrode_Type electrode;
    R1_Type r1;
    Direction direction;
    uint16_t voltage_start; // 10 bit
    uint16_t voltage_end; // 10 bit
    uint16_t num_samples; // max 1023
    uint16_t adc_samples; 
    uint32_t stardard_conductivity; // double x 1e6
    uint16_t voltage_settle_time; // in microseconds
} ProbeConfig_TypeDef;

#define CONFIG_PACKET_SIZE sizeof(ProbeConfig_TypeDef)
#define STANARD_CONDUCTIVITY (5 * 1e6) // conductivity of sodium chloride solution at 15 degrees Celsius and 1 atm pressure

#endif