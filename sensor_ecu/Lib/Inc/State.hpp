/*
 * State.hpp
 *
 *  Created on: 16 Apr 2023
 *      Author: karl
 */

#ifndef INC_STATE_HPP_
#define INC_STATE_HPP_
#include <type_traits>
#include "PodState.hpp"

class StateMaschine;

class State{
public:
	friend class StateMaschine;

	State(PodState podState) : m_podState(podState){}

	template<typename StateImplementation>
	static unsigned int getId(){
		static_assert(std::is_base_of<State, StateImplementation>());
		static unsigned int id = getNextId();
		return id;
	}

	virtual ~State() = default;

	virtual void setup() = 0;

	virtual void update() = 0;

	virtual void dispose() = 0;

	[[nodiscard]] PodState getPodState() const{
		return m_podState;
	}

protected:
	StateMaschine* m_stateMaschine = nullptr;
	bool m_active = false;

private:
	static unsigned int getNextId(){
		static unsigned int stateIdAcc;
		return stateIdAcc++;
	}
	PodState m_podState;


};


#endif /* INC_STATE_HPP_ */
