/*	AudioPlayer project for STM32 HY-Smart multimedia development board.
 *	Project made for RT and embedded systems course.
 *
 *	Author: Lukasz Bartosz
 *	Start: 8.05.2013
 *
 * Project overview
 *	uSDCard -> uC_______________________SDIO
 * uC -> WM8731 audio hardware codec___I2S(audio)/I2C(commands)
 *	Volume control______________________Potentiometer (ADC)
 * Music playback control______________Joystick (EXTIs)
 * Audio files format__________________.wav / .mp3 ???
 *
 *
 *
 *	HY-Smart PCB pinout overview
 *	LED - PB1
 *	User Key A - PC13
 *	User Key B - PB2
 */

#include "stm32f10x.h"

#define LED_ON (GPIOB->BSRR = GPIO_BSRR_BS1)
#define LED_OFF (GPIOB->BSRR = GPIO_BSRR_BR1)

int i = 0;

void SysTick_Handler(void){
	
	i++; //each ++ every 1ms
		if(i == 500){ //after 500*1ms
		LED_ON;
	}
	if(i == 1000){ //after 1000*1ms
		LED_OFF;
		i = 0;
	}
}

void init(void){
	/*
	 *	Enabling clocks for peripherals
	 * Port B:	LED
					Key B
	 * Port C:	SDIO for SD card (SDIO_D[0:3], SDIO_CK)
	 * Port D:	SDIO for SD card (SDIO_CMD)
	 *	Port E:	Joystick
	 */
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN
					 |  RCC_APB2ENR_IOPCEN
					 |  RCC_APB2ENR_AFIOEN;
	RCC->AHBENR |= RCC_AHBENR_SDIOEN;
	
	/*
	 *	SysTick init
	 *  
	 *	Clock source_________AHB/8
	 *	Exception request____Enabled
	 */	
	SysTick->LOAD = 8999; //8999 or 9000 ?????? check it
	SysTick->VAL = 0;
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE;
	SysTick->CTRL |= SysTick_CTRL_ENABLE
					  |  SysTick_CTRL_TICKINT;
	
	/*
	 *	SDIO init
	 *
	 * GPIO ports
	 * SDIO_D[0:3]_____________PC[8:11]
	 *	SDIO_CK_________________PC9
	 *	SDIO_CMD________________PD2
	 * All ports as Alternate Function with push-pull and max speed 50MHz.
	 *
	 *	HW Floc Control_________OFF
	 *	Wide bus mode___________4-wide
	 * Clock divide factor_____255+2 = 257 (lower this to speed up)
	 *	
	 *	
	 *	NOTES:
	 * need GPIO init_______Alternate function push-pull
	 * PCLK2 and SDIO_CK clock frequencies must respect the following condition:
		Freq(PCLK2) = 3/8 * Freq(SDIO_CK)
	 */
	GPIOC->CRH &= ~(GPIO_CRH_CNF8_0
				  |  GPIO_CRH_CNF9_0 
				  |  GPIO_CRH_CNF10_0 
				  |  GPIO_CRH_CNF11_0
				  |  GPIO_CRH_CNF12_0);
	GPIOC->CRH |= GPIO_CRH_CNF8_1
				  |  GPIO_CRH_MODE8
				  |  GPIO_CRH_CNF9_1
				  |  GPIO_CRH_MODE9
				  |  GPIO_CRH_CNF10_1
				  |  GPIO_CRH_MODE10
				  |  GPIO_CRH_CNF11_1
				  |  GPIO_CRH_MODE11
				  |  GPIO_CRH_CNF12_1
				  |  GPIO_CRH_MODE12;	
	
	SDIO->POWER |= SDIO_POWER_PWRCTRL; //Power-on
	SDIO->CLKCR |= SDIO_CLKCR_WIDBUS_0
					|	SDIO_CLKCR_CLKEN
					|	SDIO_CLKCR_CLKDIV;
	
	//LED init
	GPIOB->CRL &= ~GPIO_CRL_CNF1_0; //reset for push-pull configuration
	GPIOB->CRL |= GPIO_CRL_MODE1; //output speed 50MHz

	//User key A init
	
	//User key B init
	//Floating input reset state is enough
	
	//Joystick init
	
	//Potentiometer init
	
}

int main(void){
	init();
	
	while(1){

	}
}
