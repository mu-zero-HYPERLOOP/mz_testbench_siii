/*
 * propulsion_pressure.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "propulsion_pressure.hpp"

#include "AdcChannelController.hpp"
#include "MovingAverageFilter.hpp"
#include "AdcModule.hpp"
#include <cinttypes>
#include "canzero.hpp"
#include <cmath>

namespace propulsion_pressure {

constexpr AdcModule PROPULSION_PRESSURE_RESERVOIR_ADC_MODULE  = ADC_MODULE2;
constexpr uint16_t  PROPULSION_PRESSURE_RESERVOIR_ADC_RANK    = 3;
constexpr float     PROPULSION_PRESSURE_RESERVOIR_ZERO_OFFSET = -0.015;
constexpr float     PROPULSION_PRESSURE_RESERVOIR_C1          = 16 * 3.3 / (2.4 * 4095); //Line slope for pressure sensor
constexpr float     PROPULSION_PRESSURE_RESERVOIR_C2          = -16 * 0.6 / 2.4;         //Constant for pressure sensor
constexpr float     PROPULSION_PRESSURE_RESERVOIR_UPPER_LIMIT = 17.9;     //Upper limit for CAN message
constexpr float     PROPULSION_PRESSURE_RESERVOIR_LOWER_LIMIT = -1.9;     //Lower limit for CAN message

constexpr AdcModule PROPULSION_PRESSURE_PUSH_ADC_MODULE  = ADC_MODULE2;
constexpr uint16_t  PROPULSION_PRESSURE_PUSH_ADC_RANK    = 1;
constexpr float     PROPULSION_PRESSURE_PUSH_ZERO_OFFSET = -0.015;
constexpr float     PROPULSION_PRESSURE_PUSH_C1          = 16 * 3.3 / (2.4 * 4095); //Line slope for pressure sensor
constexpr float     PROPULSION_PRESSURE_PUSH_C2          = -16 * 0.6 / 2.4;         //Constant for pressure sensor
constexpr float     PROPULSION_PRESSURE_PUSH_UPPER_LIMIT = 17.9;     //Upper limit for CAN message
constexpr float     PROPULSION_PRESSURE_PUSH_LOWER_LIMIT = -1.9;     //Lower limit for CAN message

constexpr AdcModule PROPULSION_PRESSURE_RETRACT_ADC_MODULE  = ADC_MODULE2;
constexpr uint16_t  PROPULSION_PRESSURE_RETRACT_ADC_RANK    = 2;
constexpr float     PROPULSION_PRESSURE_RETRACT_ZERO_OFFSET = -0.015;
constexpr float     PROPULSION_PRESSURE_RETRACT_C1          = 16 * 3.3 / (2.4 * 4095); //Line slope for pressure sensor
constexpr float     PROPULSION_PRESSURE_RETRACT_C2          = -16 * 0.6 / 2.4;         //Constant for pressure sensor
constexpr float     PROPULSION_PRESSURE_RETRACT_UPPER_LIMIT = 17.9;     //Upper limit for CAN message
constexpr float     PROPULSION_PRESSURE_RETRACT_LOWER_LIMIT = -1.9;     //Lower limit for CAN message

constexpr bool APPLY_MOVING_AVERAGE_FILTER = false;
constexpr size_t MOVING_AVERAGE_SIZE = 10
MovingAverageFilter<MOVING_AVERAGE_SIZE> reservoirFilter;
MovingAverageFilter<MOVING_AVERAGE_SIZE> pushFilter;
MovingAverageFilter<MOVING_AVERAGE_SIZE> retractFilter;

constexpr bool LOG_FREQUENCLY = true;

AdcChannelController reservoirPressureAdc;
AdcChannelController pushPressureAdc;
AdcChannelController retractPressureAdc;


void init(){
	reservoirPressureAdc = AdcChannelController(PROPULSION_PRESSURE_RESERVOIR_ADC_MODULE, PROPULSION_PRESSURE_RESERVOIR_ADC_RANK);
	pushPressureAdc = AdcChannelController(PROPULSION_PRESSURE_PUSH_ADC_MODULE, PROPULSION_PRESSURE_PUSH_ADC_RANK);
	retractPressureAdc = AdcChannelController(PROPULSION_PRESSURE_RETRACT_ADC_MODULE, PROPULSION_PRESSURE_RETRACT_ADC_RANK);

	for(size_t i = 0; i < MOVING_AVERAGE_SIZE; i++){

	float reservoirPressure = readPressure(reservoirPressureAdc, PROPULSION_PRESSURE_RESERVOIR_C1,
			PROPULSION_PRESSURE_RESERVOIR_C2, PROPULSION_PRESSURE_RESERVOIR_ZERO_OFFSET,
			PROPULSION_PRESSURE_RESERVOIR_LOWER_LIMIT, PROPULSION_PRESSURE_RESERVOIR_UPPER_LIMIT);

	if(APPLY_MOVING_AVERAGE_FILTER){
		reservoirFilter.addValue(reservoirPressure);
		reservoirPressure = reservoirFilter.get();
	}


	float pushPressure = readPressure(pushPressureAdc, PROPULSION_PRESSURE_PUSH_C1,
			PROPULSION_PRESSURE_PUSH_C2, PROPULSION_PRESSURE_PUSH_ZERO_OFFSET,
			PROPULSION_PRESSURE_PUSH_LOWER_LIMIT, PROPULSION_PRESSURE_PUSH_UPPER_LIMIT);

	if(APPLY_MOVING_AVERAGE_FILTER){
		pushFilter.addValue(pushPressure);
		pushPressure = pushFilter.get();
	}


	float retractPressure = readPressure(retractPressureAdc, PROPULSION_PRESSURE_RETRACT_C1,
			PROPULSION_PRESSURE_RETRACT_C2, PROPULSION_PRESSURE_RETRACT_ZERO_OFFSET,
			PROPULSION_PRESSURE_RETRACT_LOWER_LIMIT, PROPULSION_PRESSURE_RETRACT_UPPER_LIMIT);
	if(APPLY_MOVING_AVERAGE_FILTER){
		retractFilter.addValue(retractPressure);
		retractPressure = retractFilter.get();
	}
	}

}

inline float readPressure(AdcChannelController& adc, float c1, float c2, float zeroOffset,
		lower, upper){
	uint16_t avalue = adc.get();
	float pressure = c1 * avalue + c2 + zeroOffset;
	pressure = std::max(lower, std::min(upper, pressure));
	return pressure;

}

void update(){
	// read pressure sensor.
	float reservoirPressure = readPressure(reservoirPressureAdc, PROPULSION_PRESSURE_RESERVOIR_C1,
			PROPULSION_PRESSURE_RESERVOIR_C2, PROPULSION_PRESSURE_RESERVOIR_ZERO_OFFSET,
			PROPULSION_PRESSURE_RESERVOIR_LOWER_LIMIT, PROPULSION_PRESSURE_RESERVOIR_UPPER_LIMIT);

	if(APPLY_MOVING_AVERAGE_FILTER){
		reservoirFilter.addValue(reservoirPressure);
		reservoirPressure = reservoirFilter.get();
	}


	float pushPressure = readPressure(pushPressureAdc, PROPULSION_PRESSURE_PUSH_C1,
			PROPULSION_PRESSURE_PUSH_C2, PROPULSION_PRESSURE_PUSH_ZERO_OFFSET,
			PROPULSION_PRESSURE_PUSH_LOWER_LIMIT, PROPULSION_PRESSURE_PUSH_UPPER_LIMIT);

	if(APPLY_MOVING_AVERAGE_FILTER){
		pushFilter.addValue(pushPressure);
		pushPressure = pushFilter.get();
	}


	float retractPressure = readPressure(retractPressureAdc, PROPULSION_PRESSURE_RETRACT_C1,
			PROPULSION_PRESSURE_RETRACT_C2, PROPULSION_PRESSURE_RETRACT_ZERO_OFFSET,
			PROPULSION_PRESSURE_RETRACT_LOWER_LIMIT, PROPULSION_PRESSURE_RETRACT_UPPER_LIMIT);
	if(APPLY_MOVING_AVERAGE_FILTER){
		retractFilter.addValue(retractPressure);
		retractPressure = retractFilter.get();
	}


	// update od.
	OD_PressureReservoir_set(reservoirPressure);
	OD_PressurePush_set(pushPressure);
	OD_PressureRetract_set(retractPressure);

	if(LOG_FREQUENCLY){
		can::Message<can::messages::Track_SDO_Resp> reservoirMsg;
		reservoirMsg.set<can::signals::Track_OD_PressureReservoir>(OD_PressureReservoir_get());
		reservoirMsg.send();

		can::Message<can::messages::Track_SDO_Resp> pushMsg;
		pushMsg.set<can::signals::Track_OD_PressurePush>(OD_PressurePush_get());
		pushMsg.send();

		can::Message<can::messages::Track_SDO_Resp> retractMsg;
		retractMsg.set<can::signals::Track_OD_PressureRetract>(OD_PressureRetract_get());
		retractMsg.send();
	}
}

}
