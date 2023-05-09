/*
 * StateMaschine.hpp
 *
 *  Created on: 16 Apr 2023
 *      Author: karl
 */

#ifndef INC_STATEMASCHINE_HPP_
#define INC_STATEMASCHINE_HPP_

#include "State.hpp"
#include "FreeRTOS.h"
#include <type_traits>
#include "StateMaschineMemory.hpp"
#include "cmsis_os.h"
#include "main.h"

class StateMaschine {
public:
	template<typename StateMaschineMemory_t>
	explicit StateMaschine(StateMaschineMemory_t* memory) {
		m_states = memory->m_states;
		m_statesSize = memory->size();
	}
	template<typename StateImplementation>
	void registerState(StateImplementation &state) {
		static_assert(std::is_base_of<State, StateImplementation>::value);
		unsigned int id = State::getId<StateImplementation>();
		state.m_stateMaschine = this;
		m_states[id] = &state;
	}

	template<typename STATE>
	bool wasPreviousState(){
		static_assert(std::is_base_of<State, STATE>::value);
		unsigned int id = State::getId<STATE>();
		return m_prevState == id;
	}

	template<typename DestState>
	void setState() {
		static_assert(std::is_base_of<State, DestState>::value);
		taskENTER_CRITICAL();
		m_states[m_currentState]->m_active = false;
		m_prevState = m_currentState;
		unsigned int id = State::getId<DestState>();
		if (m_running == false) {
			Error_Handler();
		}
		m_states[id]->m_active = true;
		m_nextState = id;
		taskEXIT_CRITICAL();
	}

	template<typename STATE>
	const State& getState(){
		static_assert(std::is_base_of<State, STATE>::value);
		return *(m_states[State::getId<STATE>()]);
	}

	template<typename StartState>
	void start() {
		static_assert(std::is_base_of<State, StartState>::value);
		if(m_running == true) Error_Handler();
		m_currentState = State::getId<StartState>();
		m_nextState = m_currentState;
		m_prevState = m_currentState;
		m_states[m_nextState]->m_active = true;
		m_running = true;
		m_states[m_currentState]->setup();
		while (m_running) {
			if (m_currentState != m_nextState) {
				taskENTER_CRITICAL();
				m_states[m_currentState]->dispose();
				m_states[m_nextState]->setup();
				m_currentState = m_nextState;
				taskEXIT_CRITICAL();
			}
			m_states[m_currentState]->update();
		}
	}
	void stop() {
		m_running = false;
	}
private:
	State **m_states = nullptr;
	size_t m_statesSize;
	unsigned int m_prevState = 0;
	unsigned int m_currentState = 0;
	unsigned int m_nextState = 0;
	bool m_running = false;
	bool m_initalized = false;
};

#endif /* INC_STATEMASCHINE_HPP_ */
