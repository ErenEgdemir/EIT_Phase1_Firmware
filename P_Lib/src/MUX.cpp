/*
 * MXP.cpp
 *
 *  Created on: Feb 14, 2026
 *      Author: erenegdemir
 */
#include "MUX.hpp"
#include <cstdint>

MUX::MUX(GPIO_TypeDef* enPort, uint16_t enPin, GPIO_TypeDef* mxpPort,
		uint16_t mxpPin0, uint16_t mxpPin1,
		uint16_t mxpPin2, uint16_t mxpPin3)
:
		_mxpPort(mxpPort),
		_enPort(enPort),
		_enPin(enPin),
		_mxpPin0(mxpPin0),
		_mxpPin1(mxpPin1),
		_mxpPin2(mxpPin2),
		_mxpPin3(mxpPin3)
{}


void MUX::s0Low(void){HAL_GPIO_WritePin(_mxpPort, _mxpPin0, GPIO_PIN_RESET);}
void MUX::s0High(void){HAL_GPIO_WritePin(_mxpPort, _mxpPin0, GPIO_PIN_SET);}

void MUX::s1Low(void){HAL_GPIO_WritePin(_mxpPort, _mxpPin1, GPIO_PIN_RESET);}
void MUX::s1High(void){HAL_GPIO_WritePin(_mxpPort, _mxpPin1, GPIO_PIN_SET);}

void MUX::s2Low(void){HAL_GPIO_WritePin(_mxpPort, _mxpPin2, GPIO_PIN_RESET);}
void MUX::s2High(void){HAL_GPIO_WritePin(_mxpPort, _mxpPin2, GPIO_PIN_SET);}

void MUX::s3Low(void){HAL_GPIO_WritePin(_mxpPort, _mxpPin3, GPIO_PIN_RESET);}
void MUX::s3High(void){HAL_GPIO_WritePin(_mxpPort, _mxpPin3, GPIO_PIN_SET);}

void MUX::enable(void){HAL_GPIO_WritePin(_enPort, _enPin, GPIO_PIN_RESET);}
void MUX::disable(void){HAL_GPIO_WritePin(_enPort, _enPin, GPIO_PIN_SET);}

void MUX::init(void)
{
	disable();
	HAL_Delay(50);
	enable();
}

void MUX::setPin(uint8_t pin)
{
	if((pin & 0x1)) s0High();
	else s0Low();
	if((pin & 0x2)) s1High();
	else s1Low();
	if((pin & 0x4)) s2High();
	else s2Low();
	if((pin & 0x8)) s3High();
	else s3Low();
}
