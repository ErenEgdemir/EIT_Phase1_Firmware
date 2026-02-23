/*
 * eit_types.h
 *
 *  Created on: Feb 23, 2026
 *      Author: erenegdemir
 */

#ifndef INC_EIT_TYPES_H_
#define INC_EIT_TYPES_H_

#include <stdint.h>

typedef struct {
    uint16_t map_id;
    uint32_t fs_hz;
    uint32_t f0_hz;
    uint16_t blank_periods;
    uint16_t int_periods;
} eit_cfg_t;

#endif /* INC_EIT_TYPES_H_ */
