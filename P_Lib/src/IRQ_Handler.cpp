/*
 * IRQ_Handler.c
 *
 *  Created on: Feb 15, 2026
 *      Author: erenegdemir
 */
#include <IRQ_Handler.hpp>
#include <string.h>

#include "app.hpp"
#include "stm32f4xx_hal_tim.h"

CaptureEvents CpEvents;


extern "C" {

void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		CpEvents.halfCapCallback_pll = true;

	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM5 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
	  CpEvents.fullCapCallback_pll = true;
	  CpEvents.fullCapCallback_fsm = true;
	  CpEvents.debugSend = true;
	  CpEvents.t = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
	  CpEvents.ndtr = __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
       
    }
}

}










