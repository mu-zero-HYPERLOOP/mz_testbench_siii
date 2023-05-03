/*
 * DistanceSensor.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "AdcChannelController.hpp"
#include "peripheral_config.hpp"

class DistanceSensor {
public:
	explicit DistanceSensor(const DistanceSensorConfig &config) :
			m_adcChannel(config.m_adc.m_module, config.m_adc.m_rank) {
	}
	DistanceSensor(DistanceSensor&) = delete;
	DistanceSensor(DistanceSensor&&) = delete;
	DistanceSensor& operator=(DistanceSensor&) = delete;
	DistanceSensor& operator=(DistanceSensor&&) = delete;


	float read() {
		//TODO implement correct distance mapping.
		return m_adcChannel.get() / 4095.0;
	}
private:
	AdcChannelController m_adcChannel;
};
