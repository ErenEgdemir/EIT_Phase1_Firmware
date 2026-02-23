/*
 * fsm.cpp
 *
 *  Created on: Feb 18, 2026
 *      Author: erenegdemir
 */
#include "fsm.hpp"
#include "MeasFsm.hpp"
#include "com_handler.h"

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
		_sub->applyCfg(_in.cfg.fs_hz, _in.cfg.f0_hz,
				_in.cfg.blank_periods, _in.cfg.int_periods);
		_sub->requestStart();
		changeState(State::MEAS);
	}
}

void FSM::meas()
{
	if (_in.stop && _sub) {
	        _sub->requestStop();
	        changeState(State::IDLE);
	        return;
	    }

	if(_sub) _sub->tick();

    if(_in.MeasDone) {
        memcpy(_tx_amp, _in.amp, sizeof(_tx_amp)); // latch
        _tx_pending = true;
        changeState(State::SENDING);
    }

}

void FSM::sending() {
	if (_tx_pending) {
		EIT_SendData(_tx_amp, MEAS_COUNT);
		_tx_pending = false;
		changeState(State::IDLE);
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


