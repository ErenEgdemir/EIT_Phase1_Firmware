/*
 * subFsm.cpp
 *
 *  Created on: Feb 18, 2026
 *      Author: erenegdemir
 */
#include "MeasFsm.hpp"



#define WIN_N				512
#define NOC_BIT				32
#define MAP_SIZE			10

#define REF_V				3.3f
#define ADC_MAX				4096
#define GAIN				1

#define HOW_FREQ			25000
#define PHS_SHFT			0

static MUX MUX_HOWLAND_P(MUX1_EN_GPIO_Port, MUX1_EN_Pin, MUX1_S0_GPIO_Port,
				MUX1_S0_Pin, MUX1_S1_Pin, MUX1_S2_Pin, MUX1_S3_Pin);
static MUX MUX_HOWLAND_N(MUX2_EN_GPIO_Port, MUX2_EN_Pin, MUX2_S0_GPIO_Port,
				MUX2_S0_Pin, MUX2_S1_Pin, MUX2_S2_Pin, MUX2_S3_Pin);
static MUX MUX_MEAS_P(MUX3_EN_GPIO_Port, MUX3_EN_Pin, MUX3_S0_GPIO_Port,
				MUX3_S0_Pin, MUX3_S1_Pin, MUX3_S2_Pin, MUX3_S3_Pin);
static MUX MUX_MEAS_N(MUX4_EN_GPIO_Port, MUX4_EN_Pin, MUX4_S0_GPIO_Port,
				MUX4_S0_Pin, MUX4_S1_Pin, MUX4_S2_Pin, MUX4_S3_Pin);

uint16_t adc_buf[BUF_LEN];

extern AD9833 DDS;

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
		ADC_HandleTypeDef* hadc1, TIM_HandleTypeDef* htim2):
		_n_of_elc(n_of_elc),
		_settling_period(settling_period),
		_hadc1(hadc1),
		_htim2(htim2)

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

void MEASFSM::applyCfg(uint32_t fs_hz, uint32_t f0_hz, uint16_t blank_periods, uint16_t int_periods)
{
    _fs_hz = fs_hz;
    _f0_hz = f0_hz;
    _blank_periods = blank_periods;
    _int_periods = int_periods;
    _cfg_valid = true;

    // phase increment (NOC.h içindeki fonksiyon varsa onu kullan)
    ctx.phsInc = calc_phase_increment(_f0_hz, _fs_hz, NOC_BIT);
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

	MUX_HOWLAND_P.init();
	MUX_HOWLAND_N.init();
	MUX_MEAS_P.init();
	MUX_MEAS_N.init();

	DDS.init();
	DDS.setFrequency(_cfg_valid ? _f0_hz : HOW_FREQ);
	DDS.setPhase(PHS_SHFT);
	DDS.start();

	ctx.st = MeasState::SWITCHING;
	return;

}

void MEASFSM::handleSwitching(void)
{



	if(ctx.measIndx > _n_of_elc - 3)
	{
		ctx.injIndx++;
		ctx.measIndx = 0;

		if(ctx.injIndx >= _n_of_elc)
		{
			ctx.injIndx = 0;
			ctx.measIndx = 0;
			return;
		}

		ctx.howland_p = ctx.injIndx;
		ctx.howland_n = prev(ctx.howland_p);
		ctx.measure_p = next(ctx.howland_p);
		ctx.measure_n = next(ctx.measure_p);

		disableHowlandMUXs();
		disableMeasMUXs();
		MUX_HOWLAND_P.setPin(ctx.howland_p);
		MUX_HOWLAND_N.setPin(ctx.howland_n);
		MUX_MEAS_P.setPin(ctx.measure_p);
		MUX_MEAS_N.setPin(ctx.measure_n);
		enableHowlandMUXs();
		enableMeasMUXs();

		ctx.measIndx += 2;
	}

	if(!ctx.injStart)
	{
		ctx.howland_p = ctx.injIndx;
		ctx.howland_n = prev(ctx.howland_p);
		ctx.measure_p = next(ctx.howland_p);
		ctx.measure_n = next(ctx.measure_p);

		disableHowlandMUXs();
		disableMeasMUXs();
		MUX_HOWLAND_P.setPin(ctx.howland_p);
		MUX_HOWLAND_N.setPin(ctx.howland_n);
		MUX_MEAS_P.setPin(ctx.measure_p);
		MUX_MEAS_N.setPin(ctx.measure_n);
		enableHowlandMUXs();
		enableMeasMUXs();

		ctx.measIndx += 2;

		ctx.injStart = true;
	}
	else
	{
		ctx.measure_p = next(ctx.measure_p);
		ctx.measure_n = next(ctx.measure_p);

		disableMeasMUXs();
		MUX_MEAS_P.setPin(ctx.measure_p);
		MUX_MEAS_N.setPin(ctx.measure_n);
		enableMeasMUXs();

		ctx.measIndx++;
	}

	ctx.st = MeasState::SETTLING;
	return;
}

void MEASFSM::handleSettlingIRQ(void)
{
	if(!(ctx.st == MeasState::SETTLING)) return;

	if(ctx.settling_count >= _settling_period)
	{
		ctx.settlingDone = true;
		ctx.settling_count = 0;
	}
	else ctx.settling_count++;

}

void MEASFSM::handleSettling(void)
{
	if(ctx.settlingDone)
	{
		ctx.st = MeasState::INTEGRATE;
		ctx.settlingDone = false;
		return;
	}
}

void MEASFSM::handleIntegrateTimer_IRQ(uint32_t writePos)
{
	if(ctx.integrateArmed == true){
		writePos = BUF_LEN - writePos;
		ctx.winStartPos = writePos;
		ctx.zcDone = true;
	}

}

void MEASFSM::handleIntegrate(void)
{
	ctx.integrateArmed = true;
	ctx.zcCaptured = false;

	if(ctx.zcDone == true && ctx.integrateArmed == true)
	{
		ctx.zcCaptured = true;
		ctx.integrateArmed = false;

		uint32_t NDTR = __HAL_DMA_GET_COUNTER(_hadc1->DMA_Handle);

		uint32_t currentPos = BUF_LEN - NDTR;
		uint32_t distance = (currentPos - ctx.winStartPos) & (BUF_LEN - 1);

		if(distance >= WIN_N)
		{
			if((ctx.winStartPos + WIN_N) <= BUF_LEN)
			{
				memcpy(ctx.win, &adc_buf[ctx.winStartPos], WIN_N * sizeof(uint16_t));

				ctx.st = MeasState::LOCKIN;
				return;
			}
			else {
				uint16_t len1 = BUF_LEN - ctx.winStartPos;
				uint16_t len2 = WIN_N - len1;

				memcpy(ctx.win, &adc_buf[ctx.winStartPos], len1 * sizeof(uint16_t));
				memcpy(ctx.win, &adc_buf[0], len2 * sizeof(uint16_t));

				ctx.zcDone = false;
				ctx.st = MeasState::LOCKIN;
				return;

			}
		}
	}
}

void MEASFSM::handleLockIn(void)
{
	ctx.Ipp = 0.0f;
	ctx.Qpp = 0.0f;

	uint32_t sum = 0;
	for(int i = 0; i < WIN_N; i++) sum += ctx.win[i];
	float mean = (float)sum / (float)WIN_N;

	uint16_t Vmax = ctx.win[0];
	uint16_t Vmin = ctx.win[0];

	for(int i = 0; i < WIN_N; i++)
	{
		ctx.phs += ctx.phsInc;
		uint32_t sinIndx = (ctx.phs >> (NOC_BIT - MAP_SIZE));
		uint32_t cosIndx = (sinIndx + (LUT_SIZE / 4)) & (LUT_SIZE - 1);

		float x = ((float)ctx.win[i] - mean) * (REF_V / ((float)ADC_MAX) * (float)GAIN);

		if(_lut){
			ctx.Ipp += x * _lut[cosIndx];
			ctx.Qpp += x * _lut[sinIndx];
		}


		if(Vmax < ctx.win[i]) Vmax = ctx.win[i];
		if(Vmin > ctx.win[i]) Vmin = ctx.win[i];
	}

	float Isqr = ctx.Ipp * ctx.Ipp;
	float Qsqr = ctx.Qpp * ctx.Qpp;

	ctx.Mag = sqrtf(Isqr + Qsqr);
	ctx.Amp = ctx.Mag * (2.0f / (float)WIN_N);

	uint16_t VppInt = (uint16_t)(Vmax - Vmin);
	ctx.Vpp = (float)VppInt * (REF_V / (float)ADC_MAX);

	ctx.st = MeasState::STORE;
	return;
}

void MEASFSM::handleStore(void)
{


	if(ctx.storeIndx < 208){
		ctx.strAmp[ctx.storeIndx] = ctx.Amp;
		ctx.strVpp[ctx.storeIndx] = ctx.Vpp;
		ctx.storeIndx++;
		ctx.st = MeasState::SWITCHING;
	}
	else{
		ctx.st = MeasState::DONE;
		ctx.storeIndx = 0;
	}
	return;
}

void MEASFSM::handleDone(void)
{
	disableHowlandMUXs();
	disableMeasMUXs();
	DDS.stop();

	HAL_TIM_Base_Stop(_htim2);
	HAL_ADC_Stop_DMA(_hadc1);

	if (m_ops.on_done)
	{
		m_ops.on_done(m_ops.user, ctx.strAmp, MEAS_COUNT);
	}

	ctx = {0};
	ctx.st = MeasState::IDLE;
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
