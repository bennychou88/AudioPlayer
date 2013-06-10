#ifndef _WM8731_H_
#define _WM8731_H_

#define WM8731_I2C_ADDRESS								0x34
#define WM8731_LEFT_LINE_IN                    	0x00
#define WM8731_RIGHT_LINE_IN							0x01
#define WM8731_LEFT_HEADPHONE_OUT					0x02
#define WM8731_RIGHT_HEADPHONE_OUT					0x03
#define WM8731_ANALOGUE_AUDIO_PATH_CONTROL     	0x04
#define WM8731_DIGITAL_AUDIO_PATH_CONTROL			0x05
#define WM8731_POWER_DOWN_CONTROL					0x06
#define WM8731_DIGITAL_AUDIO_INTERFACE_FORMAT  	0x07
#define WM8731_SAMPLING_CONTROL                	0x08
#define WM8731_ACTIVE_CONTROL                  	0x09
#define WM8731_RESET_REGISTER                  	0x0F

void WM8731_baseInit(uint8_t I2CAddress);
void WM8731_sendData(uint8_t registerAddr, uint16_t data);

#endif //_WM8731_H_
