/*
 * AdcConvHalfCpltCallbackHandler.hpp
 *
 *  Created on: Apr 13, 2023
 *      Author: karl
 */

#ifndef INC_ADCCONVCPLTHALFCALLBACKHANDLER_HPP_
#define INC_ADCCONVCPLTHALFCALLBACKHANDLER_HPP_

#include <functional>
#include "main.h"

class AdcConvHalfCpltCallbackHandler {
public:

	static AdcConvHalfCpltCallbackHandler& getInstance() {
		static AdcConvHalfCpltCallbackHandler instance;
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
	AdcConvHalfCpltCallbackHandler() = default;
	AdcConvHalfCpltCallbackHandler(AdcConvHalfCpltCallbackHandler&) = delete;
	AdcConvHalfCpltCallbackHandler(AdcConvHalfCpltCallbackHandler&&) = delete;
	AdcConvHalfCpltCallbackHandler& operator=(AdcConvHalfCpltCallbackHandler&) = delete;
	AdcConvHalfCpltCallbackHandler& operator=(AdcConvHalfCpltCallbackHandler&&) = delete;

	std::function<void(ADC_HandleTypeDef*)> m_listeners[16];
	unsigned int m_size = 0;
};

#endif
