/*
 * IRQ_Handler.c
 *
 *  Created on: Feb 15, 2026
 *      Author: erenegdemir
 */
#include <IRQ_Handler.hpp>
#include <string.h>

#include "app.hpp"

//volatile uint8_t adc_ready = 0;
extern uint16_t adcBuf[512];


extern "C" {

uint16_t comp_buf[2][512];


volatile uint8_t wr_idx = 0;        // callback hangi buffera yazacak
volatile uint8_t ready_mask = 0;    // bit0: buf0 hazır, bit1: buf1 hazır
volatile uint32_t drop_cnt = 0;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
//    if (hadc->Instance == ADC1) {
//
//        uint8_t w = wr_idx;
////        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_14);
//
//        if (ready_mask == 0x3) { drop_cnt++; return; }
//
//        memcpy(comp_buf[w], adcBuf, sizeof(comp_buf[w]));
//        ready_mask |= (1u << w);
//        wr_idx ^= 1; // diğer buffer'a geç
//
//
//    }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
//    uint32_t t = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
	  uint32_t writePos = __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
	  SubFSM.handleIntegrateTimer_IRQ(writePos);
	  SubFSM.handleSettlingIRQ();

  }

}

}










