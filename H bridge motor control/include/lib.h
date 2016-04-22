#ifndef _LIBS_H_
#define _LIBS_H_

#include <stdint.h>

void lib_initilize_LEDs(void);
void lib_write_LEDs(uint8_t value);
uint8_t lib_read_LEDs(void);
void lib_increment_LEDs(void);
void lib_decrement_LEDs(void);
void lib_initilize_buttons(void);
void lib_initilize_interrupt_buttons(void);
uint8_t lib_read_buttons(uint8_t button_selection);
void delay(uint32_t delay_constant);
uint8_t lib_button_pressed(uint8_t button_selection);
void lib_initilize_POT(uint8_t resolution);
void lib_initilize_adc(uint8_t resolution);
uint32_t lib_sample_POT(uint8_t POT);
void lib_initilize_TIM6(void);
void lib_start_TIM6(uint16_t PSC, uint16_t ARR);
void lib_TIM6_ADC_scale(uint16_t new_PSC, uint16_t offset, uint32_t ADC_value);
void lib_acknowledge_TIM6(void);

#endif
