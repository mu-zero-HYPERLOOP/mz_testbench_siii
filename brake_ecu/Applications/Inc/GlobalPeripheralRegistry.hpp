/*
 * GlobalSensorRegister.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "SolenoidController.hpp"
#include "SDC.hpp"

class GlobalPeripheralRegistry {
private:
	GlobalPeripheralRegistry() :
			m_solenoidController(g_peripherals.m_solenoidConfig), m_sdc(
					g_peripherals.m_sdcConfig) {

	}
	GlobalPeripheralRegistry(GlobalPeripheralRegistry&) = delete;
	GlobalPeripheralRegistry(GlobalPeripheralRegistry&&) = delete;
	GlobalPeripheralRegistry& operator=(GlobalPeripheralRegistry&) = delete;
	GlobalPeripheralRegistry& operator=(GlobalPeripheralRegistry&&) = delete;
public:
	[[nodiscard]] static inline GlobalPeripheralRegistry& getInstance() {
		static GlobalPeripheralRegistry instance;
		return instance;
	}

	[[nodiscard]] inline SolenoidController& getSolenoidController() {
		return m_solenoidController;
	}

	[[nodiscard]] inline SDC& getSDC(){
		return m_sdc;
	}

private:
	SolenoidController m_solenoidController;
	SDC m_sdc;

};
