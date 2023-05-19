/*
 * NTCSensor.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "AdcModule.hpp"
#include <cinttypes>
#include <cmath>
#include "AdcChannelController.hpp"


class NTCSensor {
public:
	explicit NTCSensor(AdcModule module, uint16_t rank, float beta, float r25, float internalResistance = 100000, float supplyVoltage = 3.3)
		: m_channelController(module, rank),
		  m_r(internalResistance),
		  m_u0(supplyVoltage),
		  m_beta(beta),
		  m_r25(r25){
	}
	NTCSensor() = default;

	float getTemperaturC(bool force = false){
		return getTemperaturK(force) - t0;
	}


	float getTemperaturK(bool force = false){
		uint16_t avalue = m_channelController.get(force);
		printf("avalue = %u\n", avalue);
		/*
		 *   3.3V
		 *    |
		 *    R  (internal resistor on the shield)
		 *    |
		 *   R_NTC --.
		 *    |      | U = avalue * 3.3 / 4095
		 *    GND  --'
		 *
		 * solve for R_NTC
		 */
		float r_ntc = m_r / ((4095.0/avalue) - 1);
		float temperature = 1.0 / (std::log(r_ntc / m_r25) / m_beta + t25_inv);
		return temperature;
	}

private:
	AdcChannelController m_channelController;
	float m_r;
	float m_u0;
	float m_beta;
	float m_r25;
	static constexpr float t25_inv = 1.0 / 298.15;
	static constexpr float t0 = 273.15;

};
