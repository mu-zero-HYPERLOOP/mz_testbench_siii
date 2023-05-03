/*
 * GlobalPeripheralRegistry.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "SDC.hpp"
#include "ExternalMdbDistanceSensor.hpp"
#include <cinttypes>

class GlobalPeripheralRegistry {
private:
	GlobalPeripheralRegistry() :
			m_sdc(g_peripherals.m_sdcConfig), m_mdbDistanceSensor {
					ExternalMdbDistanceSensor(
							g_peripherals.m_mdbDistanceConfig[0]),
					ExternalMdbDistanceSensor(
							g_peripherals.m_mdbDistanceConfig[1]),
					ExternalMdbDistanceSensor(
							g_peripherals.m_mdbDistanceConfig[2]),
					ExternalMdbDistanceSensor(
							g_peripherals.m_mdbDistanceConfig[3]),
					ExternalMdbDistanceSensor(
							g_peripherals.m_mdbDistanceConfig[4]),
					ExternalMdbDistanceSensor(
							g_peripherals.m_mdbDistanceConfig[5])
	} {

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

	[[nodiscard]] inline SDC& getSDC() {
		return m_sdc;
	}

	[[nodiscard]] inline ExternalMdbDistanceSensor& getMdbDistanceSensor(
			unsigned int idx) {
		return m_mdbDistanceSensor[idx];
	}

	[[nodiscard]] inline ExternalMdbDistanceSensor* getMdbDistanceSensors(){
		return m_mdbDistanceSensor;
	}

	[[nodiscard]] constexpr size_t getMdbDistanceSensorCount() {
		return sizeof(m_mdbDistanceSensor) / sizeof(ExternalMdbDistanceSensor);
	}

private:
	SDC m_sdc;
	ExternalMdbDistanceSensor m_mdbDistanceSensor[6];
};
