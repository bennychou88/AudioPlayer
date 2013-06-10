#include "stm32f10x.h"
#include "wm8731.h"

void WM8731_baseInit(uint8_t I2CAddress){
	WM8731_sendData(WM8731_RESET_REGISTER, 0x0000);
	WM8731_sendData(WM8731_LEFT_HEADPHONE_OUT, 0x0000);
	WM8731_sendData(WM8731_RIGHT_HEADPHONE_OUT, 0x0000);
	WM8731_sendData(WM8731_ANALOGUE_AUDIO_PATH_CONTROL, 0x0000);
	WM8731_sendData(WM8731_DIGITAL_AUDIO_PATH_CONTROL, 0x0000);
	WM8731_sendData(WM8731_POWER_DOWN_CONTROL, 0x0000);
	WM8731_sendData(WM8731_SAMPLING_CONTROL, 0x0000);
	WM8731_sendData(WM8731_DIGITAL_AUDIO_INTERFACE_FORMAT, 0x0000);
	WM8731_sendData(WM8731_ACTIVE_CONTROL, 0x0000);
}

void WM8731_sendData(uint8_t registerAddr, uint16_t data){
	uint32_t dummy;
	uint16_t dataToSend = (registerAddr << 9) | data;
	
	I2C2->CR1 |= I2C_CR1_START;
	while(!(I2C2->SR1 & I2C_SR1_SB));
	dummy = I2C2->SR1;
	I2C2->DR = WM8731_I2C_ADDRESS;
	while(!(I2C2->SR1 & I2C_SR1_ADDR));
	dummy = I2C2->SR1;
	dummy = I2C2->SR2;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->DR = (dataToSend >> 8); // & 0xFF;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
	I2C2->DR = dataToSend; // & 0xFF;
	while(!(I2C2->SR1 & I2C_SR1_TXE));
	while(!(I2C2->SR1 & I2C_SR1_BTF));
	I2C2->CR1 |= I2C_CR1_STOP;
}
