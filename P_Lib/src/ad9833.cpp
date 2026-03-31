/*
 * ad9833.cpp
 *
 *  Created on: Feb 13, 2026
 *      Author: erenegdemir
 */

#include "ad9833.hpp"

static constexpr uint16_t CTRL_B28	= (1u << 13);
static constexpr uint16_t CTRL_RST 	= (1u << 8);

static constexpr uint16_t FREQ0_ADDR = 0x4000;
static constexpr uint16_t PHASE0_ADDR = (1u << 15) | (1u << 14);

AD9833::AD9833(SPI_HandleTypeDef* hspi, GPIO_TypeDef* fsyncPort,
		uint16_t fsyncPin, uint32_t mclk_hz)
:_hspi(hspi),
 _fsyncPort(fsyncPort),
 _fsyncPin(fsyncPin),
 _mclk(mclk_hz)
{}

void AD9833::fsyncLow()  { HAL_GPIO_WritePin(_fsyncPort, _fsyncPin, GPIO_PIN_RESET); }
void AD9833::fsyncHigh() { HAL_GPIO_WritePin(_fsyncPort, _fsyncPin, GPIO_PIN_SET);   }


void AD9833::write16b(uint16_t w)
{
	uint8_t tx[2];
	tx[0] = (uint8_t)(w >> 8);
	tx[1] = (uint8_t)(w & 0xFF);



	fsyncLow();
//	delay_us(7);
	HAL_SPI_Transmit(_hspi, tx, 2, 100);
//	delay_us(7);
	fsyncHigh();
//	delay_us(100);
}

void AD9833::init(void)
{

	write16b(CTRL_B28 | CTRL_RST);

}

void AD9833::start(void)
{
	write16b(CTRL_B28 & ~CTRL_RST);
}

void AD9833::stop(void)
{
	write16b(CTRL_B28 | CTRL_RST);
}


void AD9833::setFrequency(uint32_t f_hz)
{
	uint32_t ftw = (uint32_t)(((uint64_t)f_hz << 28) / _mclk);

//	uint32_t ftw = 0x10c7;
	uint16_t LSB14 = (uint16_t)(ftw & 0x3fff);
	uint16_t MSB14 = (uint16_t)((ftw >> 14) & 0x3fff);

	write16b(FREQ0_ADDR | LSB14);
	HAL_Delay(10);
	write16b(FREQ0_ADDR | MSB14);

//	write16b(0x4001);
//	write16b(0x4000);

}

void AD9833::setPhase(uint16_t phs)
{
	uint16_t PHS = (uint16_t)(phs & 0x7ff);
	write16b(PHASE0_ADDR | PHS);
}











