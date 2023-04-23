/*
 * GPIOExtiController.hpp
 *
 *  Created on: Apr 14, 2023
 *      Author: karl
 */

#ifndef INC_GPIOEXTICONTROLLER_HPP_
#define INC_GPIOEXTICONTROLLER_HPP_

#include "main.h"
#include <cstdint>
#include <functional>

#include "GPIOExtiCallbackHandler.hpp"

class GPIOExtiController {
public:
	explicit GPIOExtiController(GPIO_TypeDef *port, uint16_t pin) :
			m_port(port), m_pin(pin) {
		m_extiIsrId = GPIOExtiCallbackHandler::getInstance().registerCallback(
				[&](uint16_t pin) {
					if (m_extiCallback != nullptr && m_pin == pin) {
						GPIO_PinState state = HAL_GPIO_ReadPin(m_port, m_pin);
						m_extiCallback(state == GPIO_PIN_SET);
					}
				});
	}
	~GPIOExtiController() {
		GPIOExtiCallbackHandler::getInstance().unregisterCallback(m_extiIsrId);
	}
	void setExtiCallback(std::function<void(bool)> extiCallback) {
		m_extiCallback = extiCallback;
	}
	void resetExtiCallback() {
		m_extiCallback = nullptr;
	}
private:
	GPIO_TypeDef *m_port;
	uint16_t m_pin;
	unsigned int m_extiIsrId;
	std::function<void(bool)> m_extiCallback = nullptr;
};

#endif /* INC_GPIOEXTICONTROLLER_HPP_ */
