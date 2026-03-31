/*
 * subFsm.cpp
 *
 *  Created on: Feb 18, 2026
 *      Author: erenegdemir
 */
#include "MeasFsm.hpp"
#include "eit_types.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
#include "nco.hpp"
#include "pll.hpp"
#include "IRQ_Handler.hpp"
#include <cmath>
#include <cstring>



#define WIN_N				512
#define NOC_BIT				32
#define MAP_SIZE			10

#define REF_V				3.3f
#define ADC_MAX				4096
#define GAIN				1

#define HOW_FREQ			25000
#define ADC_FREQ			800000
#define PHS_SHFT			0


#define F_TIM				60000000

static MUX MUX_HOWLAND_P(MUX1_EN_GPIO_Port, MUX1_EN_Pin, MUX1_S0_GPIO_Port,
				MUX1_S0_Pin, MUX1_S1_Pin, MUX1_S2_Pin, MUX1_S3_Pin);
static MUX MUX_HOWLAND_N(MUX2_EN_GPIO_Port, MUX2_EN_Pin, MUX2_S0_GPIO_Port,
				MUX2_S0_Pin, MUX2_S1_Pin, MUX2_S2_Pin, MUX2_S3_Pin);
static MUX MUX_MEAS_P(MUX3_EN_GPIO_Port, MUX3_EN_Pin, MUX3_S0_GPIO_Port,
				MUX3_S0_Pin, MUX3_S1_Pin, MUX3_S2_Pin, MUX3_S3_Pin);
static MUX MUX_MEAS_N(MUX4_EN_GPIO_Port, MUX4_EN_Pin, MUX4_S0_GPIO_Port,
				MUX4_S0_Pin, MUX4_S1_Pin, MUX4_S2_Pin, MUX4_S3_Pin);

static NCO NCO;
PLL PLL;
extern CaptureEvents CpEvents;

uint16_t adc_buf[BUF_LEN];

float k = 0;
uint32_t dt = 0;
extern AD9833 DDS;

uint32_t t_now = 0;
uint32_t t_prev = 0;
uint32_t dt_tick = 0;
uint32_t f0 = 0;

const MEASFSM::StateHandler MEASFSM::kHandlers[] = {
		&MEASFSM::handleIdle,
		&MEASFSM::handleStart,
		&MEASFSM::handleSwitching,
		&MEASFSM::handleSettling,
		&MEASFSM::handleIntegrate,
		&MEASFSM::handleLockIn,
		&MEASFSM::handleStore,
		&MEASFSM::handleDone
};

MEASFSM::MEASFSM(uint16_t n_of_elc, uint8_t settling_period,
		ADC_HandleTypeDef* hadc1, TIM_HandleTypeDef* htim2, TIM_HandleTypeDef* htim4):
		_n_of_elc(n_of_elc),
		_settling_period(settling_period),
		_hadc1(hadc1),
		_htim2(htim2),
		_htim4(htim4)

{}



void MEASFSM::disableHowlandMUXs(void)
{
	MUX_HOWLAND_P.disable();
	MUX_HOWLAND_N.disable();
}

void MEASFSM::enableHowlandMUXs(void)
{
	MUX_HOWLAND_P.enable();
	MUX_HOWLAND_N.enable();
}

void MEASFSM::disableMeasMUXs(void)
{
	MUX_MEAS_P.disable();
	MUX_MEAS_N.disable();
}

void MEASFSM::enableMeasMUXs(void)
{
	MUX_MEAS_P.enable();
	MUX_MEAS_N.enable();
}

uint16_t MEASFSM::next(uint16_t prevElc)
{
	return (prevElc + 1) & (_n_of_elc - 1);
}

uint16_t MEASFSM::prev(uint16_t nextElc)
{
	return (nextElc - 1) & (_n_of_elc - 1);
}

void MEASFSM::setOps(const Ops& ops)
{
	m_ops = ops;
}

void MEASFSM::applyCfg(eit_cfg_t cfg)
{
    _fs_hz = cfg.fs_hz;
    _f0_hz = cfg.f0_hz;
    _blank_periods = cfg.blank_periods;
    _int_periods = cfg.int_periods;
	PLL.setConfig(cfg);
    _cfg_valid = true;
}
void MEASFSM::applyMapTable(eit_map_t* m)
{
	if(m)
	{
		memcpy(_map, m, sizeof(_map));
	}
	
}

void MEASFSM::requestStart() { _start_req = true; }
void MEASFSM::requestStop()  { _stop_req = true;  }

void MEASFSM::handleIdle()
{
    if (!_start_req) return;

    _start_req = false;

    ctx.st = MeasState::START;

    _settling_period = _cfg_valid ? _blank_periods : _settling_period;
}

void MEASFSM::handleStart(void)
{


	HAL_ADC_Start_DMA(_hadc1, (uint32_t*)adc_buf, BUF_LEN);
	HAL_TIM_Base_Start(_htim2);
	

	HAL_Delay(500);

	MUX_HOWLAND_P.init();
	MUX_HOWLAND_N.init();
	MUX_MEAS_P.init();
	MUX_MEAS_N.init();

	NCO.init();
	NCO.setPhaseInc(NCO.calcPhaseIncrement(_f0_hz, _fs_hz, 32));
	PLL.init();

	ctx.rep_count = 16;
	_injectIndx = 0;

	ctx.st = MeasState::SWITCHING;
	return;

}

void MEASFSM::handleSwitching(void)
{

	MUX_HOWLAND_P.setPin(_map[_injectIndx].i_p);
	MUX_HOWLAND_N.setPin(_map[_injectIndx].i_n);
	MUX_MEAS_P.setPin(_map[_injectIndx].v_p);
	MUX_MEAS_N.setPin(_map[_injectIndx].v_n);
	_injectIndx++;
	if(_injectIndx >= 208) _injectIndx = 0;
	ctx.st = MeasState::SETTLING;
	return;
}

void MEASFSM::handleSettlingIRQ()
{
	if(!(CpEvents.fullCapCallback_fsm)) return;
	if(!(ctx.st == MeasState::SETTLING)) return;

	if((ctx.settling_count) * 16 >= _blank_periods) //16 periyotta bir callback high veriyor.
	{
		ctx.settlingDone = true;
		ctx.settling_count = 0;
	}
	else ctx.settling_count++;
	CpEvents.fullCapCallback_fsm = false;
}

void MEASFSM::handleSettling(void)
{
	handleSettlingIRQ();
	if(ctx.settlingDone)
	{
		ctx.st = MeasState::INTEGRATE;
		ctx.settlingDone = false;

		ctx.zcDone = false;
		ctx.zcCaptured = false;
		ctx.integrateArmed = true;

		ctx.st = MeasState::INTEGRATE;
		return;
	}
}

void MEASFSM::handleIntegrateTimer_IRQ(uint32_t ndtr, uint32_t t)
{
	if(!(CpEvents.fullCapCallback_fsm)) return;
	if(ctx.st != MeasState::INTEGRATE) return;
	if(!ctx.integrateArmed) return;
	if(ctx.zcDone) return;


	uint32_t t_now = __HAL_TIM_GET_COUNTER(_htim4);

	uint32_t dtick = 0;
	uint32_t dpos = 0;
	static uint32_t t_prev = 0;
	uint32_t t_edge = t;
	uint32_t T_exp = 0;

	T_exp = (uint32_t)F_TIM / (uint32_t)_f0_hz;
	
	if(t_prev == 0) {
		t_prev = t_edge;
		return;
	}
	dt = t_edge - t_prev;
	t_prev = t_edge;
	if(dt < (T_exp / 2)) return;	
	dtick = t_now - t_edge;

	dpos = (uint32_t)(((uint64_t)dtick * _fs_hz + (F_TIM/2)) / F_TIM);

	uint32_t writePos = BUF_LEN - ndtr;
	uint32_t winStartPos = (writePos + BUF_LEN - (dpos % BUF_LEN)) % BUF_LEN;
	ctx.winStartPos = (winStartPos + 1) & (BUF_LEN - 1);

	ctx.zcDone = true;
	ctx.zcCaptured = true;
	ctx.integrateArmed = false;

	CpEvents.fullCapCallback_fsm = false;
	CpEvents.halfCapCallback_fsm = false;
	
	
}

void MEASFSM::handleIntegrate(void) {

	handleIntegrateTimer_IRQ(CpEvents.ndtr, CpEvents.t);
	
	if(!ctx.zcDone) return;

	const uint32_t N = (uint32_t)(_int_periods * 32);

	uint32_t ndtr = __HAL_DMA_GET_COUNTER(_hadc1->DMA_Handle);
	uint32_t currentPos = BUF_LEN - ndtr;

	uint32_t distance = (currentPos - ctx.winStartPos) & (BUF_LEN - 1);

	if(distance < N) return;

	if(ctx.winStartPos + N <= BUF_LEN) {
		memcpy(ctx.win, &adc_buf[ctx.winStartPos], N * sizeof(uint16_t));
	}
	else {
		uint32_t len1 = BUF_LEN - ctx.winStartPos;
		uint32_t len2 = N - len1;

		memcpy(ctx.win, &adc_buf[ctx.winStartPos], len1 * sizeof(uint16_t));
		memcpy(ctx.win + len1, &adc_buf[0], len2 * sizeof(uint16_t));
	}

	ctx.zcDone = false;
	ctx.st = MeasState::LOCKIN;
	NCO.setPhaseInc(PLL.getPhaseInc());
	NCO.setPhaseCorr(PLL.getPhaseCorr());
}

void MEASFSM::handleLockIn(void)
{
	ctx.phs = 0;

	ctx.Ipp = 0.0f;
	ctx.Qpp = 0.0f;

	uint32_t sum = 0;
	for(int i = 0; i < (_int_periods * 32); i++) sum += ctx.win[i];
	float mean = (float)sum / (float)(_int_periods * 32);

	uint16_t Vmax = ctx.win[0];
	uint16_t Vmin = ctx.win[0];

	for(int i = 0; i < (_int_periods * 32); i++)
	{
		float x = ((float)ctx.win[i] - mean) * (REF_V / ((float)ADC_MAX) * (float)GAIN);

		
		ctx.Ipp += x * NCO.getCos();
		ctx.Qpp += x * NCO.getSine();

		NCO.run();

		if(Vmax < ctx.win[i]) Vmax = ctx.win[i];
		if(Vmin > ctx.win[i]) Vmin = ctx.win[i];

	}
	ctx.IppNormalized = ctx.Ipp * (2.0f / (float)(_int_periods * 32));
	ctx.QppNormalized = ctx.Qpp * (2.0f / (float)(_int_periods * 32));
	float Isqr = ctx.Ipp * ctx.Ipp;
	float Qsqr = ctx.Qpp * ctx.Qpp;

	ctx.Mag = sqrtf(Isqr + Qsqr);
	ctx.Amp = ctx.Mag * (2.0f / (float)(_int_periods * 32));

	uint16_t VppInt = (uint16_t)(Vmax - Vmin);
	ctx.Vpp = (float)VppInt * (REF_V / (float)ADC_MAX);
	k = ctx.Vpp / ctx.Amp;

	ctx.st = MeasState::STORE;

	return;
}

void MEASFSM::handleStore(void)
{
	if(ctx.storeIndx < 208){
		// ctx.strAmp[ctx.storeIndx] = ctx.Amp;
		// ctx.strVpp[ctx.storeIndx] = ctx.Vpp;
		ctx.strIpp[ctx.storeIndx] = ctx.IppNormalized;
		ctx.strQpp[ctx.storeIndx] = ctx.QppNormalized;
		ctx.storeIndx++;
		ctx.st = MeasState::SWITCHING;
	}
	else{
		ctx.st = MeasState::DONE;
		ctx.storeIndx = 0;
	}
	HAL_TIM_IC_Start_IT(_htim4, TIM_CHANNEL_1);
	return;
}

void MEASFSM::handleDone(void)
{
	disableHowlandMUXs();
	disableMeasMUXs();

	if (m_ops.on_done)
	{
		m_ops.on_done(m_ops.user, ctx.strIpp, ctx.strQpp, MEAS_COUNT);
	}

	ctx = {0};
	ctx.st = MeasState::START;
	return;
}

void MEASFSM::abortToIdle()
{
    disableHowlandMUXs();
    disableMeasMUXs();
    DDS.stop();

    HAL_TIM_Base_Stop(_htim2);
    HAL_ADC_Stop_DMA(_hadc1);

    ctx.st = MeasState::IDLE;
}

void MEASFSM::tick(void)
{
    if (_stop_req) {
        _stop_req = false;
        abortToIdle();
        return;
    }

	const auto idx = static_cast<uint8_t>(ctx.st);

	if(idx >= static_cast<uint8_t>(MeasState::COUNT)) {
		ctx.st = MeasState::START;
		return;
	}

	(this->*kHandlers[idx])();
}
