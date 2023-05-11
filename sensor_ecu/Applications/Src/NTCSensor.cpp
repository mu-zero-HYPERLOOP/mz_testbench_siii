/*
 * NTCSensor.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */

#include "NTCSensor.hpp"
#include <cmath>

NTCSensor::NTCSensor(NTCTemperaturSensorConfig config) :
		m_analogInput(config.m_adc.m_module, config.m_adc.m_rank), m_config(config) {
}

float NTCSensor::getTemperaturC(){
	return getTemperaturK() - 273.15;
}

float NTCSensor::getTemperaturK(){
	uint16_t avalue = m_analogInput.get();
	float voltage = avalue * 3.3 / 4095;
	float U_NTC = voltage; //test this with the actual shield.
	float R_NTC = (U_NTC * m_config.m_R) / (m_config.m_U0 - U_NTC);
	float temperatur = 1.0 / (std::log(R_NTC / m_config.m_r25) / m_config.m_beta + (1.0 / 298.15));
	return temperatur;
}
