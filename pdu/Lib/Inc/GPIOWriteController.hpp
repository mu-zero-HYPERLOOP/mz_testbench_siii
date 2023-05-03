/*
 * gpio_controller.cpp
 *
 *  Created on: Apr 12, 2023
 *      Author: karl
 */

#ifndef INC_GPIO_CONTROLLER_CPP_
#define INC_GPIO_CONTROLLER_CPP_

#include "main.h"

class GPIOWriteController {
public:
	explicit GPIOWriteController(GPIO_TypeDef *port, uint16_t pin) :
			m_port(port), m_pin(pin) {
	}
	void set() {
		HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_SET);
	}
	void write(bool value) {
		if (value)
			set();
		else
			reset();
	}
	void reset() {
		HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_RESET);
	}
	void toggle() {
		HAL_GPIO_TogglePin(m_port, m_pin);
	}
private:
	GPIO_TypeDef *m_port;
	uint16_t m_pin;
};

#endif /* INC_GPIO_CONTROLLER_CPP_ */
