/*
 * PressureSensor.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include "peripheral_config.hpp"
#include "AdcChannelController.hpp"

class PressureSensor {
public:
	explicit PressureSensor(const PressureConfig& config)
		: m_adcChannel(config.m_adc.m_module, config.m_adc.m_rank){

	}

	[[nodiscard]] inline float getPressure(){
		//TODO do proper convertion.
		return m_adcChannel.get() / 4095.0;
	}
private:
	AdcChannelController m_adcChannel;
};
