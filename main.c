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
#include "wm8731.h"

#define LED_ON		(GPIOB->BSRR = GPIO_BSRR_BS1)
#define LED_OFF	(GPIOB->BSRR = GPIO_BSRR_BR1)

volatile int i = 0; //in use
volatile int j = 0;
volatile uint16_t k = 0;

void SysTick_Handler(void){
	i++; //each ++ every 1ms
	if(i == 500){ //after 500*1ms
		//LED_ON;
	}
	if(i == 1000){ //after 1000*1ms
		//LED_OFF;
		i = 0;
	}
}

void I2S_playTestSound(void){
	while(!(SPI2->SR & SPI_SR_TXE));
	if(SPI2->SR & SPI_SR_CHSIDE) SPI2->DR = 0xFFFF;
	else	SPI2->DR = 0xFFFF;
}

void peripheralsInit(void){
	/*
	 *	Enabling clocks for peripherals
	 * Port B:	LED
					Key B
					I2S -> Audio hardware codec
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
	//SysTick->LOAD = 8999; //8999 or 9000 ?????? check it
	//SysTick->VAL = 0;
	//SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE;
	//SysTick->CTRL |= SysTick_CTRL_ENABLE
	//						|  SysTick_CTRL_TICKINT;
	
	/*
	 *	SDIO init
	 *
	 * Pinout
	 * SDIO_D[0:3]_____________PC[8:11]
	 *	SDIO_CK_________________PC9
	 *	SDIO_CMD________________PD2
	 * All pins as Alternate Function with push-pull and max speed 50MHz.
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
	
					  
	
	/*
	 *	I2S init
	 *
	 *	Configuration mode		Master - transmit
	 * Standard						Phillips standard
	 * Data length					16-bit
	 * Channel length				16-bit
	 * Master clock output		Disabled
	 * I2SDIV						140
	 * I2SODD						1
	 * Steady state clock polarity	Low level
	 *
	 */	
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	GPIOB->CRH &= ~(GPIO_CRH_CNF12_0
				  |  GPIO_CRH_CNF13_0 
				  |  GPIO_CRH_CNF15_0);
	GPIOB->CRH |= GPIO_CRH_CNF12_1
				  |  GPIO_CRH_MODE12
				  |  GPIO_CRH_CNF13_1
				  |  GPIO_CRH_MODE13
				  |  GPIO_CRH_CNF15_1
				  |  GPIO_CRH_MODE15;
	SPI2->I2SPR |= 0x000D;
//					|	SPI_I2SPR_ODD;
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SMOD
					  |  SPI_I2SCFGR_I2SCFG_1;
//					  |  SPI_I2SCFGR_DATLEN_0
//					  |  SPI_I2SCFGR_CHLEN;
	SPI2->CR2 |= SPI_CR2_TXEIE;
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE; //enable
	
	/*
	 *	I2C init
	 *
	 *	Pinout
	 *	SCL__________PB10
	 * SDA__________PB11
	 *	All pins as Alternate Function with open-drain and max speed 50MHz
	 *
	 * WM8731 I2C write address: 011010
	 *	Peripheral clock frequency			
	 *	
	 */
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
 	
	GPIOB->CRH |= GPIO_CRH_CNF10
				  |  GPIO_CRH_MODE10
				  |  GPIO_CRH_CNF11
				  |  GPIO_CRH_MODE11;
	
	I2C2->CR1 |= I2C_CR1_SWRST;
	I2C2->CR1 &= ~I2C_CR1_SWRST;
	I2C2->CR2 |= I2C_CR2_FREQ_2
				 |  I2C_CR2_FREQ_5;
	I2C2->CCR |= 320;
	I2C2->TRISE |= 10;
	I2C2->CR1 |= I2C_CR1_PE;
	
	//LED init
	GPIOB->CRL &= ~GPIO_CRL_CNF1_0; //reset for push-pull configuration
	GPIOB->CRL |= GPIO_CRL_MODE1; //output speed 50MHz

	//User key A init
	
	//User key B init
	//Floating input reset state is enough
	
	//Joystick init
	
	//Potentiometer init
	
	//Enable interrupts
	NVIC_EnableIRQ(I2C2_EV_IRQn);
	NVIC_EnableIRQ(SPI2_IRQn);
}


void I2C2_EV_IRQHandler(void){
}

void SPI2_IRQHandler(void){
	j++;
	if(j==5000) LED_OFF;
	else if(j==10000){LED_ON;j=0;}
	if(SPI2->SR & SPI_SR_CHSIDE) SPI2->DR = 0xFFFF;
	else	SPI2->DR = 0xFFFF;
}


int main(void){
	peripheralsInit();
	WM8731_baseInit(WM8731_I2C_ADDRESS);

	while(1){
		
	}
}
