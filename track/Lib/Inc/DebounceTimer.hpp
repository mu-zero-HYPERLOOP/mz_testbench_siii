/*
 * DebounceTimer.hpp
 *
 *  Created on: Apr 16, 2023
 *      Author: karl
 */

#ifndef INC_DEBOUNCETIMER_HPP_
#define INC_DEBOUNCETIMER_HPP_

#include "tim.h"
#include "main.h"
#include "TimPeriodElapsedCallbackHandler.hpp"

class DebounceTimer {
public:
	explicit DebounceTimer(TIM_HandleTypeDef* htim) : m_htim(htim){
		m_timerCallbackId = TimPeriodElapsedCallbackHandler::getInstance().registerCallback([&](TIM_HandleTypeDef* htim){
			if(htim == m_htim){
				periodElapsedCallback();
			}
		});
	}

	~DebounceTimer(){
		TimPeriodElapsedCallbackHandler::getInstance().unregisterCallback(m_timerCallbackId);
	}

	bool bounce(){
		if(m_bouncing){
			return false;
		}else{
			m_bouncing = true;
			HAL_TIM_Base_Start_IT(m_htim);
			return true;
		}
	}

private:
	void periodElapsedCallback(){
		HAL_TIM_Base_Stop_IT(m_htim);
		m_bouncing = false;
	}

	TIM_HandleTypeDef* m_htim;
	unsigned int m_timerCallbackId;
	bool m_bouncing = false;

};



#endif /* INC_DEBOUNCETIMER_HPP_ */
