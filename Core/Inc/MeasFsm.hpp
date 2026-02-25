/*
 * subFsm.hpp
 *
 *  Created on: Feb 18, 2026
 *      Author: erenegdemir
 */

#ifndef INC_MEASFSM_HPP_
#define INC_MEASFSM_HPP_

#include <cstdint>
#include "MUX.hpp"
#include "ad9833.hpp"

#define WIN_N			512
#define BUF_LEN			4096
#define LUT_SIZE		1024
#define MEAS_COUNT		208


extern "C" {

#include <string.h>
#include "main.h"
#include "stm32f4xx_hal.h"
#include "NOC.h"
#include "com_handler.h"

}

enum class MeasState: uint8_t {
	IDLE = 0,
	START,
	SWITCHING,
	SETTLING,
	INTEGRATE,
	LOCKIN,
	STORE,
	DONE,
	COUNT
};



struct MeasCtx {


	float Ipp = 0.0f;
	float Qpp = 0.0f;
	float Mag = 0.0f;
	float Amp = 0.0f;
	float Vpp = 0.0f;
	float lut[LUT_SIZE];

	float strAmp[MEAS_COUNT];
	float strVpp[MEAS_COUNT];

	uint32_t phs = 0;
	uint32_t phsInc = 0;

	eit_cfg_t cfg;

	uint16_t winStartPos = 0;
	uint16_t win[WIN_N];

	uint16_t howland_p = 0;
	uint16_t howland_n = 0;
	uint16_t measure_p = 0;
	uint16_t measure_n = 0;

	uint16_t injIndx;
	uint16_t measIndx;

	bool injStart = false;

	bool settlingDone = false;
	uint16_t settling_count = 0;

	bool integrateArmed = false;
	bool zcDone = false;
	bool zcCaptured = false;

	uint16_t storeIndx = 0;

	MeasState st = MeasState::IDLE;

};

class MEASFSM {
public:
	MEASFSM(uint16_t n_of_elc, uint8_t settling_period, ADC_HandleTypeDef* hadc1,
			TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim4);

	void applyCfg(uint32_t fs_hz, uint32_t f0_hz, uint16_t blank_periods, uint16_t int_periods);
    void requestStart();
	void requestStop();

	void setLut(const float* lut) { _lut = lut; }

	using DoneCb = void(*)(void* user, const float* amp, uint32_t count);

	struct Ops {
		DoneCb on_done = nullptr;
		void* user	   = nullptr;
	};

	void setOps(const Ops& ops);

	void handleMeasurement(float* payload, uint32_t payloadSize);
	void handleSettlingIRQ(void);
	void handleIntegrateTimer_IRQ(uint32_t t);
	void handleIntegrateADC_IRQ(void);



	void tick(void);

	bool measCompleted(void);

private:
	void handleSwitching(void);

	void disableHowlandMUXs(void);
	void enableHowlandMUXs(void);

	void enableMeasMUXs(void);
	void disableMeasMUXs(void);

	uint16_t next(uint16_t prevElc);
	uint16_t prev(uint16_t nextElc);

	void handleIdle(void);
	void handleStart(void);
	void handleSettling(void);
	void handleIntegrate(void);
	void handleLockIn(void);
	void handleStore(void);
	void handleDone(void);
	void abortToIdle();

	using StateHandler = void (MEASFSM::*)();
	static const StateHandler kHandlers[];




	MeasCtx ctx = {};
	uint16_t _n_of_elc;
	uint8_t _settling_period;
	ADC_HandleTypeDef* _hadc1;
	TIM_HandleTypeDef* _htim2;
	TIM_HandleTypeDef* _htim4;

	Ops m_ops{};
	bool _start_req = false;
	bool _stop_req  = false;

	// cfg
	uint32_t _fs_hz = 800000;
	uint32_t _f0_hz = 25000;
	uint16_t _blank_periods = 10;
	uint16_t _int_periods   = 16;
	bool _cfg_valid = false;

	const float* _lut = nullptr;  // LUT pointer
};



#endif /* INC_SUBFSM_HPP_ */
