/*
 * OnBoardTemperaturSensors.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include "peripheral_config.hpp"
#include "AdcChannelController.hpp"
#include "NTCSensor.hpp"
#include <cmath>
#include "canzero.hpp"

class OnBoardSensors {
public:
	explicit OnBoardSensors(OnBoardTemperaturConfig config) :
			m_internalNTC(config.m_internalNTCConfig.m_adc.m_module,
					config.m_internalNTCConfig.m_adc.m_rank), m_externalNTC(
					config.m_externalNTCConfig.m_adc.m_module,
					config.m_externalNTCConfig.m_adc.m_rank),
					m_inputVoltage(config.m_inputVoltageConfig.m_module, config.m_inputVoltageConfig.m_rank),
					m_config(config) {
	}

	float getInternalTemperaturC(){
		uint16_t avalue = m_internalNTC.get();
		float internalTemp = (3.3f * (float) avalue/4095.0f - 0.76f) / 0.0025f + 25.0f;
		return internalTemp;
	}

	float getExternalTemperaturC(){
		uint16_t avalue = m_externalNTC.get();
		float ntcTemperature = 1.0f / (1.0f / 298.15f + 1.0f / 3380.0f * log(1.0f / (4095.0f / (float) avalue - 1.0f) )) - 273.15f;
		return ntcTemperature;
	}

	float getAverageTemperaturC(){
		float average = (getExternalTemperaturC() + getInternalTemperaturC() ) / 2.0;
		return average;
	}

	float getInputVoltage() {
		uint16_t avalue = m_inputVoltage.get();
		float inputVoltage = (float)avalue/ 4095.0f * 3.3f / 0.106464f + 0.6f;
		return inputVoltage;
	}

	void updateODs(){
		float temp = getAverageTemperaturC();
		float vBat = getInputVoltage();
		OD_BoardTemp_set(temp);
		OD_InputVoltage_set(vBat);
	}

private:
	AdcChannelController m_internalNTC;
	AdcChannelController m_externalNTC;
	AdcChannelController m_inputVoltage;
	OnBoardTemperaturConfig m_config;
};
