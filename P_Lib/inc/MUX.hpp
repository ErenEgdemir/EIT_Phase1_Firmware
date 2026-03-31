/*
 * MXP.hpp
 *
 *  Created on: Feb 14, 2026
 *      Author: erenegdemir
 */

#ifndef INC_MUX_HPP_
#define INC_MUX_HPP_

#include <cstdint>
//#include <stdio.h>

extern "C" {

#include "stm32f4xx_hal.h"

}

class MUX {
public:
	MUX(GPIO_TypeDef* enPort, uint16_t enPin, GPIO_TypeDef* mxpPort,
			uint16_t mxpPin0, uint16_t mxpPin1,
			uint16_t mxpPin2, uint16_t mxpPin3);
	void init(void);
	void enable(void);
	void disable(void);
	void setPin(uint8_t pin);

private:
	void s0Low(void);
	void s0High(void);
	void s1Low(void);
	void s1High(void);
	void s2Low(void);
	void s2High(void);
	void s3Low(void);
	void s3High(void);

	GPIO_TypeDef* _mxpPort;
	GPIO_TypeDef* _enPort;
	uint16_t _enPin;
	uint16_t _mxpPin0;
	uint16_t _mxpPin1;
	uint16_t _mxpPin2;
	uint16_t _mxpPin3;


};






#endif /* INC_MXP_HPP_ */
