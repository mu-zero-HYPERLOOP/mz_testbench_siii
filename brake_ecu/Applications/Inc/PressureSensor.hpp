/*
 * PressureSensor.hpp
 *
 *  Created on: May 9, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "AdcChannelController.hpp"
#include "AnalogInput.hpp"

class PressureSensor {
public:

	explicit PressureSensor(AdcModule module, uint16_t rank,
			float internalResistance, float zeroOffset) :
			m_currentInput(module, rank, internalResistance),
			m_zeroOffset(zeroOffset),
			m_adcChannel(module, rank)
			{
	}

	float get(bool force = false) {
		float current = m_currentInput.readCurrent(force);
		printf(" i = %f\n", current);

		 uint16_t avalue = m_adcChannel.get();
		 printf("%u / 4095\n", avalue);
		 float pressure = c1 * avalue + c2;
		 if(pressure < messageLowerLimit){
			 pressure = messageLowerLimit;
		 }
		 if(pressure > messageUpperLimit){
			 pressure = messageUpperLimit;
		 }
		 printf("delta = %f\n", (current * 1000 - 4) - pressure);
		 return pressure + m_zeroOffset;
	}

	void zeroRun(unsigned int samples) {
		double average = 0;
		for (size_t i = 0; i < samples; i++) {
			average += get(true);
		}
		printf("zero-offset = %f\n", average / samples);
	}

private:
	AnalogInput m_currentInput;
	float m_zeroOffset;

	 static constexpr float c1 = 16 * 3.3 / (2.4 * 4095); //Line slope for pressure sensor
	 static constexpr float c2 = -16 * 0.6 / 2.4;         //Constant for pressure sensor
	 static constexpr float messageUpperLimit = 17.9;     //Upper limit for CAN message
	 static constexpr float messageLowerLimit = -1.9;     //Lower limit for CAN message
	 AdcChannelController m_adcChannel;
};
