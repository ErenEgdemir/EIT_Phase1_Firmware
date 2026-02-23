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
#include "NOC.h"
#include "MeasFsm.hpp"
#include "fsm.hpp"

//#define ADC_BUF_LEN 				512
#define LUT_SIZE					1024


AD9833 DDS(&hspi1, DDS_CS_GPIO_Port, DDS_CS_Pin, 25000000);

static FSM MainFsm;
MEASFSM SubFSM(16, 16, &hadc1, &htim2);

//uint16_t adcBuf[ADC_BUF_LEN];

float lut[LUT_SIZE];

uint32_t phs = 0;
uint32_t phsInc = 0;



struct AppContext {

	volatile bool measDone = false;
	float amp[208];
};

AppContext appCtx;

static void Meas_Done_CallBack(void* user, const float* amp, uint32_t count)
{
	auto* app = static_cast<AppContext*>(user);

	memcpy(app->amp, amp, 208 * sizeof(float));
	app->measDone = true;
}


extern "C" {
	void app_setup(void)
	{

		generate_LUT(lut, LUT_SIZE);

		phsInc = calc_phase_increment(25000, ADC_FREQ, NOC_BIT);

		MEASFSM::Ops ops;
		ops.on_done = Meas_Done_CallBack;
		ops.user = &appCtx;

		SubFSM.setOps(ops);
		SubFSM.setLut(lut);

		MainFsm.setSubFsm(&SubFSM);


	}

	void app_loop(void)
	{
		EIT_FrameParser_Poll();
		CDC_TxRetry_Poll();

		FSM::Inputs in{};

		if(appCtx.measDone) {
			in.MeasDone = appCtx.measDone;
			memcpy(in.amp, appCtx.amp, 208 * sizeof(float));
		}

		in.start = EIT_TakeStartReq();
		in.stop  = EIT_TakeStopReq();
		EIT_GetCfg(&in.cfg);

		MainFsm.tick(in);

		if (in.MeasDone) {
			appCtx.measDone = false;
		}

	}
}






