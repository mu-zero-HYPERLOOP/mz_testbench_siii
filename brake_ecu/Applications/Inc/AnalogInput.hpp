/*
 * AnalogInput.hpp
 *
 *  Created on: May 14, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "AdcChannelController.hpp"
#include "AdcModule.hpp"
#include <cinttypes>
#include "estdio.hpp"

class AnalogInput {
public:
	AnalogInput(AdcModule module, uint16_t rank, float internalResistance,
			float supplyVoltage = 0) :
			m_channelController(module, rank), m_r(internalResistance), m_u0(
					supplyVoltage) {

	}

	float readCurrent(bool force = false){
		uint16_t avalue = m_channelController.get(force);
		return avalue * 3.3 / (4095 * m_r);
	}

	float readVoltage(bool force = false){
		if(m_u0 == 0){
			printf("ERROR: can't convert to voltage without knowing the supplyVoltage!");
			Error_Handler();
		}
		uint16_t avalue = m_channelController.get(force);
		float u2 = avalue * 3.3 / 4095;
		return m_u0 - u2;
	}

	float readResistance(bool force = false){
		if(m_u0 == 0){
			printf("ERROR: can't convert to resistance without knowing the supplyVoltage!");
			Error_Handler();
		}
		uint16_t avalue = m_channelController.get(force);
		float u2 = avalue * 3.3 / 4095;
		float r = m_r * (u2 - m_u0) / m_u0;
		return r;
	}


private:
	AdcChannelController m_channelController;
	float m_r;
	float m_u0;
};
