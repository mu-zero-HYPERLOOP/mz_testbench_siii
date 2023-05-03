/*
 * GlobalSensorRegister.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "DistanceSensor.hpp"
#include "PneumaticPistonController.hpp"
#include "peripheral_config.hpp"

class GlobalPeripheralRegistry {
private:
	GlobalPeripheralRegistry() : m_distanceSensor(g_peripherals.m_distanceSensor),
		m_pistonController(g_peripherals.m_pneumaticPistonConfig){

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

	[[nodiscard]] inline DistanceSensor& getDistanceSensor() {
		return m_distanceSensor;
	}

	[[nodiscard]] inline PneumaticPistonController& getPneumaticPistonController() {
		return m_pistonController;
	}

private:
	DistanceSensor m_distanceSensor;
	PneumaticPistonController m_pistonController;

};
