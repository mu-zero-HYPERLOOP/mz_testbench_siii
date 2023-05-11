/*
 * PressureSensor.hpp
 *
 *  Created on: May 9, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "AdcChannelController.hpp"

class PressureSensor {
public:

	explicit PressureSensor(AdcModule module, uint16_t rank) : m_adcChannel(module, rank){

	}

	float get(){
		uint16_t avalue = m_adcChannel.get();
		float pressure = c1 * avalue + c2;
		if(pressure < messageLowerLimit){
			pressure = messageLowerLimit;
		}
		if(pressure > messageUpperLimit){
			pressure = messageUpperLimit;
		}
		return pressure;
	}
private:
	static constexpr float c1 = 16 * 3.3 / (2.4 * 4095); //Line slope for pressure sensor
	static constexpr float c2 = -16 * 0.6 / 2.4;         //Constant for pressure sensor
	static constexpr float messageUpperLimit = 17.9;     //Upper limit for CAN message
	static constexpr float messageLowerLimit = -1.9;     //Lower limit for CAN message
	AdcChannelController m_adcChannel;
};
