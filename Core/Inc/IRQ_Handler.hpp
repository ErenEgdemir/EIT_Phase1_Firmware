/*
 * IRQ_Handler.h
 *
 *  Created on: Feb 15, 2026
 *      Author: erenegdemir
 */

#ifndef INC_IRQ_HANDLER_HPP_
#define INC_IRQ_HANDLER_HPP_

#include "stm32f4xx_hal.h"
#include <cstdint>
#include "MeasFsm.hpp"

extern MEASFSM SubFSM;

#ifdef __cplusplus
extern "C" {
#endif



extern uint16_t comp_buf[2][512];
extern volatile uint8_t ready_mask;

extern ADC_HandleTypeDef hadc1;

#ifdef __cplusplus
}
#endif



#endif /* INC_IRQ_HANDLER_HPP_ */
