#ifndef __VOLTAGE_H
#define __VOLTAGE_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "ProbeConfig.h"
#include "math.h"

/* Private defines -----------------------------------------------------------*/
/* SWITCHES */
#define GPIOSW GPIOC
#define SW_R1_Msk (SW_R1_100_Pin | SW_R1_1k_Pin | SW_R1_10k_Pin)
#define SW_Ti_Pin_Msk (SW_R1_Ti1_Pin | SW_R1_Ti2_Pin | SW_Ti1_GND_Pin | SW_Ti2_GND_Pin)
#define SW_Au_Pin_Msk (SW_R1_Au1_Pin | SW_R1_Au2_Pin | SW_Au1_GND_Pin | SW_Au2_GND_Pin)
#define SW_Shield_Pin_Msk (SW_Shield1_V_Pin | SW_Shield1_GND_Pin | SW_Shield2_V_Pin | SW_Shield2_GND_Pin)

/* DAC */
#define DAC_device_addr (0b1001000 << 1)
#define DAC_broadcast_addr (0b1000111 << 1)

#define DAC_mem_addr_status 0xd0
#define DAC_mem_addr_general_config 0xd1
#define DAC_mem_addr_med_alarm_config 0xd2
#define DAC_mem_addr_trigger 0xd3
#define DAC_mem_addr_dac_data 0x21 // needs 10 bits << 2
#define DAC_mem_addr_dac_margin_high 0x25 // needs 10 bits << 2
#define DAC_mem_addr_dac_margin_low 0x26 // needs 10 bits << 2
#define DAC_mem_addr_pmbus_operation 0x01
#define DAC_mem_addr_pmbus_status_byte 0x78
#define DAC_mem_addr_pmbus_version 0x98

#define DAC_general_config_reset 0x01f0
#define DAC_general_config_pdn_mask (0b11 << 3)
#define DAC_general_config_pdn_power_up (0b00 << 3)
#define DAC_general_config_pdn_power_down_10k (0b01 << 3)
#define DAC_general_config_pdn_power_down_HI (0b10 << 3)

#define DAC_trigger_reset 0x0008
#define DAC_trigger_config_reset (0b1 << 9)

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    ADC_CHANNEL_UNBUFF_DAC = ADC_CHANNEL_0,
    ADC_CHANNEL_DAC = ADC_CHANNEL_1,
    ADC_CHANNEL_AMP = ADC_CHANNEL_2,
    ADC_CHANNEL_SIGNAL = ADC_CHANNEL_3,
    ADC_CHANNEL_AU1 = ADC_CHANNEL_4,
    ADC_CHANNEL_AU2 = ADC_CHANNEL_5,
    ADC_CHANNEL_TI1 = ADC_CHANNEL_6,
    ADC_CHANNEL_TI2 = ADC_CHANNEL_7,
    ADC_CHANNEL_CALIB = ADC_CHANNEL_8
} ADC_Channel;

typedef struct {
    int16_t dac_input;
    int16_t dac_output;
    int16_t calib;
    int16_t measurement;
} VoltageSample_TypeDef;

/* Functions prototypes ---------------------------------------------*/
void voltage_init (void);
void us_delay(uint16_t micro_s);
void dac_init (void);
void dac_drain (void);
void dac_set_voltage(uint16_t voltage);
void dac_write (uint16_t memory_addr, uint16_t data);

void adc_set_channel (ADC_Channel channel);
uint16_t adc_average (uint16_t adc_samples);

void reset_muxs (void);
uint16_t measure_dac_voltage (uint16_t adc_samples, uint16_t voltage_settle_time);
uint16_t measure_calib_voltage (uint16_t adc_samples, uint16_t voltage_settle_time);
uint16_t measure_pin_voltage (Direction direction, Electrode_Type electrode, uint16_t adc_samples, uint16_t voltage_settle_time);
void measure_voltage_sweep (VoltageSample_TypeDef* samples, Direction direction, R1_Type r1, Electrode_Type electrode, uint16_t dac_start, uint16_t dac_stop, uint16_t num_samples, uint16_t adc_samples, uint16_t voltage_settle_time, uint16_t relaxation_time);

void measure_ac_sweep (VoltageSample_TypeDef* samples, R1_Type r1, Electrode_Type electrode, uint8_t num_waves, uint16_t num_samples, uint16_t sample_delay);

#endif // __RESISTANCE_H