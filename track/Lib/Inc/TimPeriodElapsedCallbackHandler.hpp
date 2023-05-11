/*
 * TimPeriodElapsedCallbackHandler.hpp
 *
 *  Created on: Apr 13, 2023
 *      Author: karl
 */

#ifndef INC_ADCCONVCPLTHALFCALLBACKHANDLER_HPP_
#define INC_ADCCONVCPLTHALFCALLBACKHANDLER_HPP_

#include <functional>
#include "main.h"

class TimPeriodElapsedCallbackHandler {
public:

	static TimPeriodElapsedCallbackHandler& getInstance() {
		static TimPeriodElapsedCallbackHandler instance;
		return instance;
	}

	unsigned int registerCallback(std::function<void(TIM_HandleTypeDef*)> isr) {
		unsigned int id = m_size;
		m_listeners[m_size++] = isr;
		return id;
	}

	void unregisterCallback(unsigned int callbackId) {
		m_listeners[callbackId] = m_listeners[m_size];
		m_listeners[m_size] = nullptr;
		m_size--;
	}

	void notify(TIM_HandleTypeDef *htim) {
		for (unsigned int i = 0; i < m_size; i++) {
			m_listeners[i](htim);
		}
	}

private:
	TimPeriodElapsedCallbackHandler() = default;
	TimPeriodElapsedCallbackHandler(TimPeriodElapsedCallbackHandler&) = delete;
	TimPeriodElapsedCallbackHandler(TimPeriodElapsedCallbackHandler&&) = delete;
	TimPeriodElapsedCallbackHandler& operator=(TimPeriodElapsedCallbackHandler&) = delete;
	TimPeriodElapsedCallbackHandler& operator=(
			TimPeriodElapsedCallbackHandler&&) = delete;

	std::function<void(TIM_HandleTypeDef*)> m_listeners[16];
	unsigned int m_size = 0;
};

#endif
