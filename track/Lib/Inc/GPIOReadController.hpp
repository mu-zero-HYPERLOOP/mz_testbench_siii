/*
 * GPIOReadController.hpp
 *
 *  Created on: 12 Apr 2023
 *      Author: karl
 */

#ifndef INC_GPIOREADCONTROLLER_HPP_
#define INC_GPIOREADCONTROLLER_HPP_

#include "main.h"

class GPIOReadController {
public:
	explicit GPIOReadController(GPIO_TypeDef *port, uint16_t pin) :
			m_port(port), m_pin(pin) {

	}
	bool read() {
		return HAL_GPIO_ReadPin(m_port, m_pin) == GPIO_PIN_SET;
	}
private:
	GPIO_TypeDef *m_port;
	uint16_t m_pin;
};

#endif /* INC_GPIOREADCONTROLLER_HPP_ */
