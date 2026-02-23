/*
 * ad9833.hpp
 *
 *  Created on: Feb 13, 2026
 *      Author: erenegdemir
 */

#ifndef INC_AD9833_HPP_
#define INC_AD9833_HPP_

#pragma once
#include <cstdint>

extern "C" {

#include "stm32f4xx_hal.h"

}

class AD9833 {

public:
	AD9833(SPI_HandleTypeDef* hspi, GPIO_TypeDef* fsyncPort,
			uint16_t fsyncPin, uint32_t mclk_hz = 25000000);

	void init();

	void setFrequency(uint32_t f_hz);
	void setPhase(uint16_t phs);
	void start(void);
	void stop(void);



private:
	void write16b(uint16_t w);
	void fsyncLow(void);
	void fsyncHigh(void);


	SPI_HandleTypeDef* _hspi;
	GPIO_TypeDef* _fsyncPort;
	uint16_t _fsyncPin;
	uint32_t _mclk;

};

#endif /* INC_AD9833_HPP_ */
