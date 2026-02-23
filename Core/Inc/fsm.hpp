/*
 * fsm.hpp
 *
 *  Created on: Feb 18, 2026
 *      Author: erenegdemir
 */

#ifndef INC_FSM_HPP_
#define INC_FSM_HPP_

#include <cstdint>

#define MEAS_COUNT	208

class MEASFSM;

extern "C"{
#include "com_handler.h"
#include "eit_types.h"
}

class FSM {
public:
	enum class State: uint8_t {
		IDLE = 0,
		MEAS,
		SENDING,
		FAULT,
		BORDER
	};
	struct Inputs {
		bool MeasDone = false;
		float amp[MEAS_COUNT];
		bool start = false;
		bool stop = false;
		eit_cfg_t cfg;

	};

	float _tx_amp[MEAS_COUNT];
	bool  _tx_pending = false;

	explicit FSM(State st = State::IDLE);

	void setSubFsm(MEASFSM* sub) { _sub = sub; }


	State getState() const;				//salt okunur

	void changeState(State st);

	void tick(const Inputs& in);

private:
	void idle(void);
	void meas(void);
	void sending(void);
	void fault(void);

	using StateHandler = void (FSM::*)();
	static const StateHandler Handlers[];



	State _st;
	Inputs _in{};
	MEASFSM* _sub = nullptr;

};

#endif /* INC_FSM_HPP_ */
