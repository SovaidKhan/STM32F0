#include "lib.h"
#define STM32F051
#include "stm32f0xx.h"

//Initialize LEDs to output mode
void lib_initilize_LEDs(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODER0_0;
	GPIOB->MODER |= GPIO_MODER_MODER1_0;
	GPIOB->MODER |= GPIO_MODER_MODER2_0;
	GPIOB->MODER |= GPIO_MODER_MODER3_0;
	GPIOB->MODER |= GPIO_MODER_MODER4_0;
	GPIOB->MODER |= GPIO_MODER_MODER5_0;
	GPIOB->MODER |= GPIO_MODER_MODER6_0;
	GPIOB->MODER |= GPIO_MODER_MODER7_0;

}

//Write to LEDs
void lib_write_LEDs(uint8_t value){
	GPIOB->ODR = value;
}

//Read value on LEDs
uint8_t lib_read_LEDs(void){
	uint8_t value = GPIOB->ODR;
	return value;
}

void lib_increment_LEDs(void){
    GPIOB->ODR += 1;
}   

void lib_decrement_LEDs(void){
    GPIOB->ODR -= 1;
}

//-------------------------------------------------------------------------


//Initilize push buttons
void lib_initilize_buttons(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR1_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR3_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR5_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR6_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR7_0;
}

//Initilize buttons with interrupt
void lib_initilize_interrupt_buttons(void){
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // clock for the system configurationcontroller
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // set interrupt to be sourced fromport A for EXTICR0
	EXTI->IMR |= EXTI_IMR_MR0; // un-mask the interrupt for A0
	EXTI->FTSR |= EXTI_FTSR_TR0; // enable the falling edge trigger for line 0
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}

//Read push buttons
uint8_t lib_read_buttons(uint8_t button_selection){
    
	uint8_t button_data = GPIOA->IDR;
	button_data &= button_selection;
	return button_data;
}

//kukkie delay loop, 473600 is 1 second
void delay(uint32_t delay_constant){
		*(uint32_t*)0x200000F0=delay_constant;
				while (*(uint32_t*)0x200000F0 > 0)	{
				*(uint32_t*)0x200000F0=(*(uint32_t*)0x200000F0)-1;
			}
}
//-------------------------------------------------------------------------

//Initialize the ADC
//  INPUT - R1 (0b[ALIGN][RES]000)
//			    ALIGN - 0 = Right, 1 = Left **This function will be right aligned
//			    RES - 00 = 12 bit, 01 = 10 bit, 10 = 8 bit, 11 = 6 bit
void lib_initilize_POT(uint8_t resolution){
    //Set mode to analog   
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODER5;
    GPIOA->MODER |= GPIO_MODER_MODER6;

    //Start ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    //Enable ADC
    ADC1->CR |= ADC_CR_ADEN;
    //Wait for ADC to be enabled
    while ((ADC1->ISR &= ADC_ISR_ADRDY) != ADC_ISR_ADRDY){
        
    }
    //Set resolution and alignment
    uint8_t resalign;
    if (resolution == 12){
    	resalign = 0b000;
    }
    else if (resolution == 10){
    	resalign = 0b001;
    }
    else if (resolution == 8){
        resalign = 0b010;
    }
    else {
    	resalign = 0b011;
    }
    ADC1->CFGR1 |= resalign; 
}

//Initilize ADC
void lib_initilize_adc(uint8_t resolution) {
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; 			//enable clock for ADC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; 			//enable clock for port
	GPIOA->MODER |= GPIO_MODER_MODER6; 			//set PA6 to analogue mode
	ADC1->CHSELR |= ADC_CHSELR_CHSEL6; 			// select channel 6
	if (resolution == 12){
			ADC1->CFGR1 |= 0b000; 				// resolution to 12 bit
	    }
	    else if (resolution == 10){
	    	ADC1->CFGR1 = 0b001;
	    }
	    else if (resolution == 8){
	    	ADC1->CFGR1 |= ADC_CFGR1_RES_1; 	// resolution to 8 bit
	    }
	    else {
	    	ADC1->CFGR1 |= 0b011;
	    }
	ADC1->CR |= ADC_CR_ADEN; 					// set ADEN=1 in the ADC_CR register
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);	//wait until ADRDY==1 in ADC_ISR
}

//Sample POT
//  INPUT - R1 (POT 0 = 0b100000, POT 1 = 0b1000000)
uint32_t lib_sample_POT(uint8_t POT_number){
    //Selects the correct POT
	uint8_t POT;
	if (POT_number == 1){
		POT = 0b1000000;
	}
	else {
		POT = 0b100000;
	}
    ADC1->CHSELR  = POT;
    //Start the ADC sample
    ADC1->CR |= ADC_CR_ADSTART;
    //Wait for EOC
    while((ADC1->ISR &= ADC_ISR_EOC) == ADC_ISR_EOC){
    }
    //Read sample
    return  ADC1->DR;    
}



//-------------------------------------------------------------------------

void lib_initilize_TIM6(void){
    //enable TIM6
    RCC->APB1ENR |= RCC_APB1Periph_TIM6;
    }
    
void lib_start_TIM6(uint16_t PSC, uint16_t ARR){
    TIM6->PSC = PSC;
    TIM6->ARR = ARR;
	//Buffereing the timer
	TIM6->CR1 = TIM_CR1_ARPE;
	//enable interrupt request
    TIM6->DIER |= TIM_DIER_UIE;
    //enable TIM6 IRQ in NVIC.. fix this up
    *(uint32_t*)0xE000E100 |= 0b100000000000000000;
    //start timer
    TIM6->CR1 |= TIM_CR1_CEN;
}


//offset = (t0*ADCmax + t0 - t1)/(t1 - t0)
//PSC = (t0 * 8x10^6 - offset - 1)/(offset + 1)
void lib_TIM6_ADC_scale(uint16_t new_PSC, uint16_t offset, uint32_t ADC_value){
    
    TIM6->PSC = new_PSC;
    TIM6->ARR = offset + ADC_value;
}

void lib_acknowledge_TIM6(void){
    TIM6->SR = 0x0;
}

