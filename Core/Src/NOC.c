/*
 * NOC.c
 *
 *  Created on: Feb 15, 2026
 *      Author: erenegdemir
 */
#include "NOC.h"

uint32_t calc_phase_increment(uint32_t sineFreq, uint32_t adcFreq, uint8_t nocBit)
{
	if(nocBit > 32) nocBit = 32;
	uint32_t phaseInc = (uint32_t)(((uint64_t)sineFreq << nocBit) / adcFreq);

	return phaseInc;
}

void generate_LUT(float *lut, uint16_t lutSize)
{
	for(uint16_t i = 0; i < lutSize; i++)
	{
		float rad = (float)i * ((2.0f * PI) / (float)lutSize);
		float sine = sinf(rad);

		lut[i] = sine;

	}
}

