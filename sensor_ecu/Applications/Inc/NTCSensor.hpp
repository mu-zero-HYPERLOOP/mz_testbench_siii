/*
 * NTCSensor.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "AnalogInput.hpp"
#include "AdcModule.hpp"
#include <cinttypes>
#include <cmath>


class NTCSensor {
public:
	explicit NTCSensor(AdcModule module, uint16_t rank, float internalResistance, float supplyVoltage, float b25, float r25)
		: m_analogInput(module, rank, internalResistance, supplyVoltage){

	}

	float getTemperaturC(bool force = false){
		return getTemperaturK(force) - t0;
	}

	float getTemperaturK(bool force = false){
		float r_ntc = m_analogInput.readResistance(force);
		float log = std::log(r_ntc / m_r25);
		float denom = log / m_b25 + t25_inv;
		return 1.0 / denom;
	}

private:
	AnalogInput m_analogInput;
	float m_b25;
	float m_r25;
	static constexpr float t25_inv = 1.0 / 298.15;
	static constexpr float t0 = 273.15;

};
