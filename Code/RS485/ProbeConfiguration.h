#ifndef __PROBECONFIGURATION_H
#define __PROBECONFIGURATION_H

typedef enum {
    R1_100 = 0U,
    R1_1k = 1U,
    R1_10k = 2U
} R1_Type;

typedef enum {
    Au = 0U,
    Ti = 1U,
    Au_Shielded = 2U,
} Electrode_Type;

typedef enum {
    UNIDIRECTIONAL = 0U,
    BIDIRECTIONAL = 1U,
} Direction;

typedef struct {
    Electrode_Type electrode;
    R1_Type r1;
    Direction direction;
    uint16_t voltage_start;
    uint16_t voltage_end;
    uint16_t num_samples;
    uint16_t adc_samples;
} ProbeConfig_TypeDef;

#define CONFIG_PACKET_SIZE sizeof(ProbeConfig_TypeDef)
#define DATA_PACKET_SIZE 3

#define DECIMAL_POINT 0x10
#define NEGATIVE_SIGN 0x20

#endif