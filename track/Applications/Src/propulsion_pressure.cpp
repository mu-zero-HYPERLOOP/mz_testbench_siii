/*
 * propulsion_pressure.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "propulsion_pressure.hpp"

#include "AdcChannelController.hpp"
#include "AdcModule.hpp"
#include <cinttypes>
#include "canzero.hpp"
#include <cmath>

namespace propulsion_pressure {

constexpr AdcModule PROPULSION_PRESSURE_ADC_MODULE  = ADC_MODULE2;
constexpr uint16_t  PROPULSION_PRESSURE_ADC_RANK    = 0;
constexpr float     PROPULSION_PRESSURE_ZERO_OFFSET = -0.015;
constexpr float     PROPULSION_PRESSURE_C1          = 16 * 3.3 / (2.4 * 4095); //Line slope for pressure sensor
constexpr float     PROPULSION_PRESSURE_C2          = -16 * 0.6 / 2.4;         //Constant for pressure sensor
constexpr float     PROPULSION_PRESSURE_UPPER_LIMIT = 17.9;     //Upper limit for CAN message
constexpr float     PROPULSION_PRESSURE_LOWER_LIMIT = -1.9;     //Lower limit for CAN message

AdcChannelController pressureAdc;

void init(){
	pressureAdc = AdcChannelController(PROPULSION_PRESSURE_ADC_MODULE, PROPULSION_PRESSURE_ADC_RANK);
}

void update(){
	// read pressure sensor.
	uint16_t avalue = pressureAdc.get();
	float pressure = PROPULSION_PRESSURE_C1 * avalue + PROPULSION_PRESSURE_C2 + PROPULSION_PRESSURE_ZERO_OFFSET;
	pressure = std::max(PROPULSION_PRESSURE_LOWER_LIMIT, std::min(PROPULSION_PRESSURE_UPPER_LIMIT, pressure));

	// filter pressure sensor
	float filteredPressure = pressure;

	// update od.
	OD_PropulsionPressure_set(filteredPressure);
}

}
