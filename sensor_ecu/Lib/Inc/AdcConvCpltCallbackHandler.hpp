/*
 * AdcConvCpltCallbackHandler.hpp
 *
 *  Created on: Apr 13, 2023
 *      Author: karl
 */

#ifndef INC_ADCCONVCPLTCALLBACKHANDLER_HPP_
#define INC_ADCCONVCPLTCALLBACKHANDLER_HPP_

#include <functional>
#include "main.h"

class AdcConvCpltCallbackHandler {
public:

	static AdcConvCpltCallbackHandler& getInstance() {
		static AdcConvCpltCallbackHandler instance;
		return instance;
	}

	unsigned int registerCallback(std::function<void(ADC_HandleTypeDef*)> isr) {
		unsigned int id = m_size;
		m_listeners[m_size++] = isr;
		return id;
	}

	void unregisterCallback(unsigned int callbackId) {
		m_listeners[callbackId] = m_listeners[m_size];
		m_listeners[m_size] = nullptr;
		m_size--;
	}

	void notify(ADC_HandleTypeDef *hadc) {
		for (unsigned int i = 0; i < m_size; i++) {
			m_listeners[i](hadc);
		}
	}

private:
	AdcConvCpltCallbackHandler() = default;
	AdcConvCpltCallbackHandler(AdcConvCpltCallbackHandler&) = delete;
	AdcConvCpltCallbackHandler(AdcConvCpltCallbackHandler&&) = delete;
	AdcConvCpltCallbackHandler& operator=(AdcConvCpltCallbackHandler&) = delete;
	AdcConvCpltCallbackHandler& operator=(AdcConvCpltCallbackHandler&&) = delete;

	std::function<void(ADC_HandleTypeDef*)> m_listeners[14];
	unsigned int m_size = 0;
};

#endif
