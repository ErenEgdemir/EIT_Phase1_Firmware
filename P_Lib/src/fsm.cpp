/*
 * fsm.cpp
 *
 *  Created on: Feb 18, 2026
 *      Author: erenegdemir
 */
#include "fsm.hpp"
#include "IRQ_Handler.hpp"
#include "MeasFsm.hpp"
#include "com_handler.h"
#include "eit_types.h"
#include "pll.hpp"
#include "profiler.h"

extern PLL PLL;
eit_debug_t debugFSM;

const FSM::StateHandler FSM::Handlers[] = {
		&FSM::idle,
		&FSM::meas,
		&FSM::sending,
		&FSM::fault
};

FSM::FSM(State st):
	_st(st)
{}



FSM::State FSM::getState() const
{
	return _st;
}

void FSM::changeState(State state)
{
	_st = state;
}


void FSM::idle()
{
	EIT_FrameParser_Poll();
	CDC_TxRetry_Poll();
	
	if(_in.start && _sub){
		_sub->applyCfg(_in.cfg);
		_sub->applyMapTable(_in.map);
		_sub->requestStart();
		changeState(State::MEAS);
	}
}

void FSM::meas()
{
	EIT_FrameParser_Poll();
	CDC_TxRetry_Poll();
	if (_in.stop && _sub) {
	        _sub->requestStop();
	        changeState(State::IDLE);
	        return;
	}
#if DEBUG_EN
	if(CpEvents.debugSend){
		static uint32_t i = 0;
		i++;
		if(i > 100)
		{
			PLL.getDebugData(&debugFSM);
			EIT_SendDebug(&debugFSM);
			CpEvents.debugSend = false;
			i = 0;
		}

	}
#endif

	PLL.captureBlockProcessing();
	PLL.update();
	if(_sub) _sub->tick();

    if(_in.MeasDone) {
		
		Profiler_Begin(FSM_SEND_MEASDONE_CND);
        memcpy(_tx_Ipp, _in.Ipp, sizeof(_tx_Ipp)); // latch
		memcpy(_tx_Qpp, _in.Qpp, sizeof(_tx_Qpp));
        _tx_pending = true;
        changeState(State::SENDING);
		Profiler_End(FSM_SEND_MEASDONE_CND);
    }

}

void FSM::sending() {
	if (_tx_pending) {
		EIT_SendData(_tx_Ipp, MEAS_COUNT);
		EIT_SendData(_tx_Qpp, MEAS_COUNT);
		_tx_pending = false;
		changeState(State::MEAS);
	}

}
void FSM::fault() {}

void FSM::tick(const Inputs& in)
{
	_in = in;

	const auto idx = static_cast<uint8_t>(_st);
	if(idx >= static_cast<uint8_t>(State::BORDER)) {
		changeState(State::FAULT);
		return;
	}

	(this->*Handlers[idx])();
}


