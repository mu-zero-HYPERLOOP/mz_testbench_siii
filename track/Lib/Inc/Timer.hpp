/*
 * Timer.hpp
 *
 *  Created on: Apr 15, 2023
 *      Author: karl
 */

#ifndef INC_TIMER_HPP_
#define INC_TIMER_HPP_

#include "main.h"
#include "TimPeriodElapsedCallbackHandler.hpp"
#include "estdio.hpp"

class Timer {
public:
	explicit Timer(TIM_HandleTypeDef *htim) :
			m_htim(htim) {
		m_cpltIsr = TimPeriodElapsedCallbackHandler::getInstance().registerCallback([&](TIM_HandleTypeDef* htim){
			if(m_htim == htim){
				m_overflow++;
			}
		});
	}

	void start() {
		reset();
		HAL_TIM_Base_Start_IT(m_htim);
	}

	void stop() {
		HAL_TIM_Base_Stop_IT(m_htim);
	}

	void reset() {
		m_overflow = 0;
		__HAL_TIM_SET_COUNTER(m_htim, 0);
	}

	[[nodiscard]] inline uint16_t get() {
		return __HAL_TIM_GET_COUNTER(m_htim);
	}
	[[nodiscard]] inline unsigned int overflow(){
		return m_overflow;
	}

private:
	TIM_HandleTypeDef *m_htim;
	unsigned int m_overflow = 0;
	unsigned int m_cpltIsr;
};

#endif /* INC_TIMER_HPP_ */
