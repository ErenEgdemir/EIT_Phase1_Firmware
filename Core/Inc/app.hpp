/*
 * app.hpp
 *
 *  Created on: Feb 14, 2026
 *      Author: erenegdemir
 */

#ifndef INC_APP_HPP_
#define INC_APP_HPP_

#define	ADC_FREQ		799744u
#define NOC_BIT			32u
#define SHIFT_SIZE		10     //phase mapping for LUT (1024 = 2^10)


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "com_handler.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;
//extern volatile uint8_t adc_ready;
extern uint16_t comp_buf[2][512];
extern volatile uint8_t ready_mask;



void app_setup(void);
void app_loop(void);
float map(float x, float in_min, float in_max, float out_min, float out_max);

#ifdef __cplusplus
}
#endif



#endif /* INC_APP_HPP_ */
