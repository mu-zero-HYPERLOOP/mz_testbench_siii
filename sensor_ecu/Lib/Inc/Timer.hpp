/*
 * Timer.hpp
 *
 *  Created on: Apr 15, 2023
 *      Author: karl
 */

#ifndef INC_TIMER_HPP_
#define INC_TIMER_HPP_

#include "main.h"

class Timer {
public:
	explicit Timer(TIM_HandleTypeDef *htim) :
			m_htim(htim) {
	}

	void start() {
		HAL_TIM_Base_Start_IT(m_htim);
	}

	void stop() {
		HAL_TIM_Base_Stop_IT(m_htim);
	}

	void reset() {
		__HAL_TIM_SET_COUNTER(m_htim, 0);
	}

	uint16_t get() {
		return __HAL_TIM_GET_COUNTER(m_htim);
	}

private:
	TIM_HandleTypeDef *m_htim;
};

#endif /* INC_TIMER_HPP_ */
