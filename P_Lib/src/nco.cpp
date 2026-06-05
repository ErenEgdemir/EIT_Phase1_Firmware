#include "nco.hpp"
#include <cstdint>
#include <cmath>

#define NCO_BIT		32
#define LUT_BIT		12
#define LUT_SIZE	4096

void NCO::generateLut(float *lut, uint16_t lutSize)
{
    for(uint16_t i = 0; i < lutSize; i++)
	{
	    float rad = (float)i * ((2.0f * PI) / (float)lutSize);
		float sine = sinf(rad);

		lut[i] = sine;

	}
}

uint32_t NCO::calcPhaseIncrement(uint32_t sineFreq, uint32_t adcFreq, uint8_t ncoBit)
{
    if(ncoBit > 32) ncoBit = 32;
	uint32_t phaseInc = (uint32_t)(((uint64_t)sineFreq << ncoBit) / adcFreq);

	return phaseInc;
}

void NCO::init()
{
	generateLut(LUT, 4096);
	_phase = 0;
}

void NCO::run()
{
	_phase += _phaseInc;
		
}
void NCO::setPhaseInc(uint32_t phaseInc)
{
	_phaseInc = phaseInc;
}

void NCO::setPhaseCorr(int32_t phaseCorr)
{
	_phaseCorr = phaseCorr;
}

float NCO::getSine()
{
	_phaseUse = _phase + _phaseCorr;
	_sinIndx = _phaseUse >> (NCO_BIT - LUT_BIT);
	return LUT[_sinIndx];
}

float NCO::getCos()
{
	_phaseUse = _phase + _phaseCorr;
	uint32_t baseIndx = _phaseUse >> (NCO_BIT - LUT_BIT);
	_cosIndx = (baseIndx + (LUT_SIZE / 4)) & (LUT_SIZE - 1);
	return LUT[_cosIndx];
}

uint32_t NCO::getPhase()
{
	return _phase;
}

void NCO::reset()
{
	_phase = 0;
}