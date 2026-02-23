/*
 * NOC.h
 *
 *  Created on: Feb 15, 2026
 *      Author: erenegdemir
 */

#ifndef INC_NOC_H_
#define INC_NOC_H_

#include <math.h>
#include <stdint.h>



#define PI				3.14159265358979323846f

#ifdef __cplusplus
extern "C" {
#endif

uint32_t calc_phase_increment(uint32_t sineFreq, uint32_t adcFreq, uint8_t nocBit);

void generate_LUT(float *lut, uint16_t lutSize);

#ifdef __cplusplus
}
#endif



#endif /* INC_NOC_H_ */
