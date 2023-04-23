/*
 * State.hpp
 *
 *  Created on: 16 Apr 2023
 *      Author: karl
 */

#ifndef INC_STATE_HPP_
#define INC_STATE_HPP_
#include <type_traits>

class StateMaschine;

class State{
public:
	friend class StateMaschine;
	template<typename StateImplementation>
	static unsigned int getId(){
		static_assert(std::is_base_of<State, StateImplementation>());
		static unsigned int id = getNextId();
		return id;
	}

	virtual void setup() = 0;

	virtual void update() = 0;

	virtual void dispose() = 0;

protected:
	StateMaschine* m_stateMaschine = nullptr;

private:
	static unsigned int getNextId(){
		static unsigned int stateIdAcc;
		return stateIdAcc++;
	}
};


#endif /* INC_STATE_HPP_ */
