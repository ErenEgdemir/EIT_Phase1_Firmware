/*
 * app.cpp
 *
 *  Created on: Feb 14, 2026
 *      Author: erenegdemir
 */

#include "app.hpp"
#include <math.h>
#include <string.h>
#include "MUX.hpp"
#include "ad9833.hpp"
#include "MeasFsm.hpp"
#include "com_handler.h"
#include "fsm.hpp"
#include "stm32f4xx_hal_tim.h"
#include "profiler.h"

AD9833 DDS(&hspi1, DDS_CS_GPIO_Port, DDS_CS_Pin, 24000000);

static FSM MainFsm;
MEASFSM SubFSM(16, 32, &hadc1, &htim2, &htim5);

struct AppContext {

	volatile bool measDone = false;
	float Ipp[208];
	float Qpp[208];
};

AppContext appCtx;

static void Meas_Done_CallBack(void* user, const float* Ipp, const float* Qpp, uint32_t count)
{
	Profiler_Begin(APP_MEASDONE_CALL);
	auto* app = static_cast<AppContext*>(user);

	memcpy(app->Ipp, Ipp, 208 * sizeof(float));
	memcpy(app->Qpp, Qpp, 208 * sizeof(float));
	app->measDone = true;
	Profiler_End(APP_MEASDONE_CALL);
}


extern "C" {
	void app_setup(void)
	{	
		
		MEASFSM::Ops ops;
		ops.on_done = Meas_Done_CallBack;
		ops.user = &appCtx;
		SubFSM.setOps(ops);
	
		MainFsm.setSubFsm(&SubFSM);
		DDS.init();
		DDS.setFrequency(25000);
		DDS.setPhase(0);
		DDS.start();
		HAL_TIM_Base_Start(&htim3);

	}

	void app_loop(void)
	{
		EIT_FrameParser_Poll();
		CDC_TxRetry_Poll();

		FSM::Inputs in{};

		if(appCtx.measDone) {
			Profiler_Begin(APP_MASDONE_CND);
			in.MeasDone = appCtx.measDone;
			memcpy(in.Ipp, appCtx.Ipp, 208 * sizeof(float));
			memcpy(in.Qpp, appCtx.Qpp, 208 * sizeof(float));
			Profiler_End(APP_MASDONE_CND);
		}

		in.start = EIT_TakeStartReq();
		in.stop  = EIT_TakeStopReq();
		EIT_GetCfg(&in.cfg);
		EIT_GetMapTable(in.map);

		MainFsm.tick(in);

		if (in.MeasDone) {
			appCtx.measDone = false;
		}

	}
}






