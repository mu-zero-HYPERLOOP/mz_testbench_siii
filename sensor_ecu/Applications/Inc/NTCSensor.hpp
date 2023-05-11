/*
 * NTCSensor.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include "peripheral_config.hpp"
#include "AdcChannelController.hpp"


class NTCSensor {
public:
	explicit NTCSensor(NTCTemperaturSensorConfig config);

	float getTemperaturC();

	float getTemperaturK();

private:
	AdcChannelController m_analogInput;
	NTCTemperaturSensorConfig& m_config;
};
