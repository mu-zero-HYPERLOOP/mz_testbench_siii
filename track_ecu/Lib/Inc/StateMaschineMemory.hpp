/*
 * StateMaschineMemory.hpp
 *
 *  Created on: 16 Apr 2023
 *      Author: karl
 */

#ifndef INC_STATEMASCHINEMEMORY_HPP_
#define INC_STATEMASCHINEMEMORY_HPP_

class StateMaschine; //forward declaration.

template<size_t NUMBER_OF_STATES>
class StateMaschineMemory {
public:
	friend class StateMaschine;
	explicit StateMaschineMemory(){ }
private:
	constexpr size_t size() const {return NUMBER_OF_STATES;}
	State* m_states[NUMBER_OF_STATES];
};


#endif /* INC_STATEMASCHINEMEMORY_HPP_ */
