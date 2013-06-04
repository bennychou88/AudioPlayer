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

#define LED_ON		(GPIOB->BSRR = GPIO_BSRR_BS1)
#define LED_OFF	(GPIOB->BSRR = GPIO_BSRR_BR1)

#define WM8731_I2C_ADDRESS						0x34
#define WM8731_LEFT_LINE_IN                    ((uint16_t)0x00)<<9
#define WM8731_RIGHT_LINE_IN                   ((uint16_t)0x01)<<9
#define WM8731_LEFT_HEADPHONE_OUT              ((uint16_t)0x02)<<9
#define WM8731_RIGHT_HEADPHONE_OUT             ((uint16_t)0x03)<<9
#define WM8731_ANALOGUE_AUDIO_PATH_CONTROL     ((uint16_t)0x04)<<9
#define WM8731_DIGITAL_AUDIO_PATH_CONTRL       ((uint16_t)0x05)<<9
#define WM8731_POWER_DOWN_COTROL               ((uint16_t)0x06)<<9
#define WM8731_DIGITAL_AUDIO_INTERFACE_FORMAT  ((uint16_t)0x07)<<9
#define WM8731_SAMPLING_CONTROL                ((uint16_t)0x08)<<9
#define WM8731_ACTIVE_CONTROL                  ((uint16_t)0x09)<<9
#define WM8731_RESET_REGISTER                  ((uint16_t)0x0F)<<9

volatile int i = 0;
volatile int j = 0;

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

void init(void){
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
	SysTick->LOAD = 8999; //8999 or 9000 ?????? check it
	SysTick->VAL = 0;
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE;
	SysTick->CTRL |= SysTick_CTRL_ENABLE
							|  SysTick_CTRL_TICKINT;
	
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
	 
// 						GPIOC->CRH &= ~(GPIO_CRH_CNF8_0
// 												|  GPIO_CRH_CNF9_0 
// 												|  GPIO_CRH_CNF10_0 
// 												|  GPIO_CRH_CNF11_0
// 												|  GPIO_CRH_CNF12_0);
// 						GPIOC->CRH |= GPIO_CRH_CNF8_1
// 											|  GPIO_CRH_MODE8
// 											|  GPIO_CRH_CNF9_1
// 											|  GPIO_CRH_MODE9
// 											|  GPIO_CRH_CNF10_1
// 											|  GPIO_CRH_MODE10
// 											|  GPIO_CRH_CNF11_1
// 											|  GPIO_CRH_MODE11
// 											|  GPIO_CRH_CNF12_1
// 											|  GPIO_CRH_MODE12;	
// 						
// 						SDIO->POWER |= SDIO_POWER_PWRCTRL; //Power-on
// 						SDIO->CLKCR |= SDIO_CLKCR_WIDBUS_0
// 											|	SDIO_CLKCR_CLKEN
// 											|	SDIO_CLKCR_CLKDIV; //finish this
					  
	
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
	SPI2->I2SPR |= 0x8C //I2SDIV = 140
					|	SPI_I2SPR_ODD;
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SMOD
					  |  SPI_I2SCFGR_I2SCFG_1;
	SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE; //enable
	
	/*
	 *	I2C init
	 *
	 *	Pinout
	 *	SCL__________PB100
	 * SDA__________PB11
	 *	All pins as Alternate Function with open-drain and max speed 50MHz
	 *
	 * WM8731 I2C write address: 011010
	 *	Peripheral clock frequency			2MHz
	 *	
	 */
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
// 	NVIC_EnableIRQ(I2C2_EV_IRQn);
// 	GPIOB->CRH |= GPIO_CRH_CNF10
// 				  |  GPIO_CRH_MODE10
// 				  |  GPIO_CRH_CNF11
// 				  |  GPIO_CRH_MODE11;
// 	I2C2->CR2 |= I2C_CR2_FREQ_1;
// 	I2C2->CCR |= 0x0028; //add description for this crap line
// 	I2C2->TRISE = 3;
// 	//I2C2->CR2 |= I2C_CR2_ITEVTEN;
// 	I2C2->OAR1 |= I2C_OAR1_ADD7;
// 	I2C2->CR1 |= I2C_CR1_PE;
	I2C2->CR1 |= I2C_CR1_SWRST;
	I2C2->CR1 &= ~I2C_CR1_SWRST;
	I2C2->TRISE = 37;
	I2C2->CCR = 178;
	I2C2->CR2 |= I2C_CR2_FREQ_2
				 |  I2C_CR2_FREQ_5;
	I2C2->CR1 |= I2C_CR1_PE;
	
	//LED init
	GPIOB->CRL &= ~GPIO_CRL_CNF1_0; //reset for push-pull configuration
	GPIOB->CRL |= GPIO_CRL_MODE1; //output speed 50MHz

	//User key A init
	
	//User key B init
	//Floating input reset state is enough
	
	//Joystick init
	
	//Potentiometer init
	
}

void I2C2_EV_IRQHandler(void){
	if(I2C2->SR1 & I2C_SR1_AF){
		I2C2->SR1 &= ~I2C_SR1_AF;
		LED_ON;
	}
}

void I2CSendData(uint8_t addr, uint8_t data){
	LED_ON;
	//I2C2->CR1 |= I2C_CR1_SWRST;
	//I2C2->CR1 &= ~I2C_CR1_SWRST;
	
	//I2C2->CR2 |= I2C_CR2_FREQ_1;
	//I2C2->CCR |= 0x0028; //add description for this crap line
	//I2C2->OAR1 |= I2C_OAR1_ADD7;
	
	//I2C2->CR1 |= I2C_CR1_PE;
	
	//S
	I2C2->CR1 |= I2C_CR1_START;	
	
	//EV5
	//while(!(I2C2->SR1 & I2C_SR1_SB));
	I2C2->SR1;
	I2C2->DR = WM8731_I2C_ADDRESS;
	//I2C2->CR2 |= I2C_CR2_ITEVTEN; 
	
	//EV6
	//while(!(I2C2->SR1 & I2C_SR1_ADDR));
	I2C2->SR1;
	I2C2->SR2;
	
	//EV8_1	
	I2C2->DR = addr;
	//while(I2C2->SR2 & I2C_SR2_BUSY);
	
	//Data1 / EV8
	I2C2->DR = data;
	//while(I2C2->SR2 & I2C_SR2_BUSY);
	
	//P
	
	while(1){
		if(I2C2->SR1 & (I2C_SR1_BTF | I2C_SR1_TXE)){
			I2C2->CR1 |= I2C_CR1_STOP;
			break;
		}
	}
	
	//while(I2C2->SR1 & I2C_SR1_AF);
	LED_OFF;
}

void playTestSound(void){
	while(!(SPI2->SR & SPI_SR_TXE));
	SPI2->DR = 0xFF;
	while(!(SPI2->SR & SPI_SR_TXE));
	SPI2->DR = 0x99;
}

void WM8731_sendCommand(uint16_t registerAddress, uint16_t data){
	uint32_t dummy;
	data |= registerAddress;
	I2C2->CR1 |= I2C_CR1_START;
	while(!(I2C2->SR1 | I2C_SR1_SB));
	dummy = I2C2->SR1;
	I2C2->DR = WM8731_I2C_ADDRESS;
	while(!(I2C2->SR1 | I2C_SR1_ADDR));
	dummy = I2C2->SR1;
	dummy = I2C2->SR2;
	while(!(I2C2->SR1 | I2C_SR1_TXE));
	I2C2->DR = (uint8_t)(data >> 8);
	while(!(I2C2->SR1 | I2C_SR1_TXE));
	I2C2->DR = (uint8_t)data;
	//while((I2C2->SR1 & I2C_SR1_BTF) || (!(I2C2->SR1 & I2C_SR1_TXE)));
	I2C2->CR1 |= I2C_CR1_STOP;
}

int main(void){
	init();
	
	WM8731_sendCommand(WM8731_RESET_REGISTER, 0x00);
	WM8731_sendCommand(WM8731_LEFT_HEADPHONE_OUT, 0x01C4);
	WM8731_sendCommand(WM8731_RIGHT_HEADPHONE_OUT, 0x01C4);
	WM8731_sendCommand(WM8731_ANALOGUE_AUDIO_PATH_CONTROL, 0x12);
	WM8731_sendCommand(WM8731_DIGITAL_AUDIO_PATH_CONTRL, 0x04);
	WM8731_sendCommand(WM8731_POWER_DOWN_COTROL, 0x07);
	WM8731_sendCommand(WM8731_SAMPLING_CONTROL, 0x00);
	WM8731_sendCommand(WM8731_ACTIVE_CONTROL, 0x01);

	while(1){
		LED_ON;	
		playTestSound();
	}
}
