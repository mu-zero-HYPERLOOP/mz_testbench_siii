/*
 * SolenoidController.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include "GPIOWriteController.hpp"
#include "peripheral_config.hpp"

class SolenoidController {
public:
	explicit SolenoidController(const SolenoidConfig &config) :
			m_gpioController(config.m_gpio.m_port, config.m_gpio.m_pin) {

	}
	SolenoidController(SolenoidController&) = delete;
	SolenoidController(SolenoidController&&) = delete;
	SolenoidController& operator=(SolenoidController&) = delete;
	SolenoidController& operator=(SolenoidController&&) = delete;
	void activate() {
		m_gpioController.set();
	}

	void deactivate() {
		m_gpioController.reset();
	}
private:
	GPIOWriteController m_gpioController;
};
