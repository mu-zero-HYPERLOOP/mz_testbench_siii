/*
 * cooling_pressure.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "cooling_pressure.hpp"
#include "AdcModule.hpp"
#include <cinttypes>
#include "canzero.hpp"
#include <cmath>
#include "AdcChannelController.hpp"

namespace cooling_pressure {

static AdcChannelController reservoirPressureAdc;

constexpr AdcModule RESERVOIR_PRESSURE_ADC_MODULE  = ADC_MODULE2;
constexpr uint16_t  RESERVOIR_PRESSURE_ADC_RANK    = 3;
constexpr float     RESERVOIR_PRESSURE_ZERO_OFFSET = -0.015;
constexpr float     RESERVOIR_PRESSURE_C1          = 16 * 3.3 / (2.4 * 4095); //Line slope for pressure sensor
constexpr float     RESERVOIR_PRESSURE_C2          = -16 * 0.6 / 2.4;         //Constant for pressure sensor
constexpr float     RESERVOIR_PRESSURE_UPPER_LIMIT = 17.9;     //Upper limit for CAN message
constexpr float     RESERVOIR_PRESSURE_LOWER_LIMIT = -1.9;     //Lower limit for CAN message

void init(){
	reservoirPressureAdc = AdcChannelController(RESERVOIR_PRESSURE_ADC_MODULE, RESERVOIR_PRESSURE_ADC_RANK);
}

void update(){
	// read pressure sensor.
	uint16_t avalue = reservoirPressureAdc.get();
	printf("avalue = %u\n", avalue);
	float reservoirPressure = RESERVOIR_PRESSURE_C1 * avalue + RESERVOIR_PRESSURE_C2 + RESERVOIR_PRESSURE_ZERO_OFFSET;
	reservoirPressure = std::max(RESERVOIR_PRESSURE_LOWER_LIMIT, std::min(RESERVOIR_PRESSURE_UPPER_LIMIT, reservoirPressure));

	// filter pressure sensor.
	float filteredReservoirPressure = reservoirPressure;

	// update od.
	OD_CoolingPressure_set(filteredReservoirPressure);
}

}
