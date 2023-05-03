/*
 * PneumaticPistonController.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "GPIOWriteController.hpp"
#include "peripheral_config.hpp"

class PneumaticPistonController {
public:
	explicit PneumaticPistonController(
			const PneumaticPistonControllerConfig &config) :
			m_gpioController(config.m_gpio.m_port, config.m_gpio.m_pin) {

	}

	void activate() {
		//TODO check if activate and deactivate implementations have to be swapped.
		m_gpioController.set();
	}

	void deactivate(){
		m_gpioController.reset();
	}

private:
	GPIOWriteController m_gpioController;
};
