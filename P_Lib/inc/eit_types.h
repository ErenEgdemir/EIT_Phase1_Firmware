/*
 * eit_types.h
 *
 *  Created on: Feb 23, 2026
 *      Author: erenegdemir
 */

#ifndef INC_EIT_TYPES_H_
#define INC_EIT_TYPES_H_

#include <stdint.h>

#define DEBUG_EN   1

typedef struct {
    uint16_t map_id;
    uint32_t fs_hz;
    uint32_t f0_hz;
    uint16_t blank_periods;
    uint16_t int_periods;

    float freqKpFine;
    float freqKiFine;
    float phaseKpFine;
    float freqKpCoarse;
    float freqKiCoarse;
    float phaseKpCoarse;

    uint32_t PhaseIdeal;
    float FreqErrorMaxFine;
    uint32_t PhaseErrorMaxFine;

    float FreqErrorMaxCoarse;
    float FreqErrorMinCoarse;
    uint32_t PhaseErrorMaxCoarse;
    uint32_t PhaseErrorMinCoarse;
    
} eit_cfg_t;

typedef struct {
    uint8_t i_p;
    uint8_t i_n;
    uint8_t v_p;
    uint8_t v_n;
} eit_map_t;

typedef struct {
    uint32_t phase;
    uint32_t phaseInc;
    int32_t phaseCorr;
    float freqError;
    float freqi;
    int32_t phaseError;
    uint8_t st;
    uint32_t measuredFreq;
} eit_debug_t;

#endif /* INC_EIT_TYPES_H_ */
