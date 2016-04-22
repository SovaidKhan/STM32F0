

#define STM32F051

#include <stdint.h>
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include "lib.h"

void main(void);
void TIM3_IRQHandler(void);
uint32_t counter = 0;

void main(void)
{
//	This code toggles pin B4 in order to generate a square wave.
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // enable clock to the timer
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable clock for LEDs
	GPIOB->MODER |= GPIO_MODER_MODER4_1; //set B4 to alternate function
	GPIOB->AFR[0] |= 0x01 << (4*4); //set AFR to AF1 for B4
	/* (1) Set prescaler to 3, so APBCLK/4 i.e 12MHz */
	/* (2) Set ARR = 12000*/
	/* (3) Set CCRx = ARR, as timer clock is 12MHz, an event occurs each 1 ms */
	/* (4) Select toggle mode on OC1 (OC1M = 011*/
	/* (5) Select active high polarity on OC1 (CC1P = 0, reset value), enable the
	output on OC1 (CC1E = 1)*/
	/* (6) Enable counter */
	TIM3->PSC |= 1; /* (1) */
	TIM3->ARR = 4800; /* (2) */
	TIM3->CCR1 = 30000; /* (3) */
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; /* (4) */
	TIM3->CCER |= TIM_CCER_CC1E; /* (5)*/
	TIM3->CR1 |= TIM_CR1_CEN; /* (6) */
	for(;;){
	}
}
void TIM3_IRQHandler(void) {
	//Read the capture counter which clears the CC1ICF
		//counter = TIM2->CCR1;
	GPIOB->ODR = 0x01;
	}
