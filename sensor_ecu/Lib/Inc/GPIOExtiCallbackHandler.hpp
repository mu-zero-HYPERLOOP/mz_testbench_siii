/*
 * GPIOExtiCallbackHandler.hpp
 *
 *  Created on: Apr 14, 2023
 *      Author: karl
 */

#ifndef INC_GPIOEXTICALLBACKHANDLER_HPP_
#define INC_GPIOEXTICALLBACKHANDLER_HPP_

#include <functional>
#include "main.h"

class GPIOExtiCallbackHandler {
public:

	static GPIOExtiCallbackHandler& getInstance() {
		static GPIOExtiCallbackHandler instance;
		return instance;
	}

	unsigned int registerCallback(std::function<void(uint16_t)> isr) {
		unsigned int id = m_size;
		m_listeners[m_size++] = isr;
		return id;
	}

	void unregisterCallback(unsigned int callbackId) {
		m_listeners[callbackId] = m_listeners[m_size];
		m_listeners[m_size] = nullptr;
		m_size--;

	}

	void notify(uint16_t pin) {
		for (unsigned int i = 0; i < m_size; i++) {
			m_listeners[i](pin);
		}
	}

private:
	GPIOExtiCallbackHandler() = default;
	GPIOExtiCallbackHandler(GPIOExtiCallbackHandler&) = delete;
	GPIOExtiCallbackHandler(GPIOExtiCallbackHandler&&) = delete;
	GPIOExtiCallbackHandler& operator=(GPIOExtiCallbackHandler&) = delete;
	GPIOExtiCallbackHandler& operator=(GPIOExtiCallbackHandler&&) = delete;

	std::function<void(uint16_t)> m_listeners[16];
	unsigned int m_size = 0;
};

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	GPIOExtiCallbackHandler::getInstance().notify(pin);
}

#endif /* INC_GPIOEXTICALLBACKHANDLER_HPP_ */
