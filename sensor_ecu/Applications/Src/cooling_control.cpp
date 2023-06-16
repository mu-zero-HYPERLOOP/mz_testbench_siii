/*
 * cooling_control.hpp
 *
 *  Created on: 11 May 2023
 *      Author: karl
 */

#include <brake_ecu_remote.hpp>
#include <pdu_remote.hpp>
#include "cooling_controll.hpp"
#include "clu_remote.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "canzero.hpp"
#include "AdcModule.hpp"
#include "AdcChannelController.hpp"
#include "MovingAverageFilter.hpp"

namespace cooling {

constexpr float THRESHOLD = 30;

static MODE s_mode;
static MODE s_nextMode = MODE::ADAPTIV;
static osMutexId_t s_modeMutex = osMutexNew(NULL);

constexpr float RESERVOIR_PRESSURE_HIGH = 5;
constexpr float RESERVOIR_PRESSURE_LOW = 2;

constexpr float RESERVOIR_TEMPERATURE_LOW = 30;
constexpr float MAGNET_TEMPERATURE_LOW = 30;

constexpr float RESERVOIR_NTC_NOMINAL_RESISTANCE = 8500;
constexpr float RESERVOIR_NTC_NOMINAL_TEMPERATURE = 273.15 + 25;
constexpr float RESERVOIR_NTC_BETA = 4200;
constexpr float RESERVOIR_NTC_INTERNAL_RESISTOR = 100000;
constexpr AdcModule RESERVOIR_NTC_ADC_MODULE = ADC_MODULE2;
constexpr uint16_t RESERVOIR_NTC_ADC_RANK = 0;

pdu::HpChannel COOLING_PUMP_CHANNEL = pdu::HP_CHANNEL3;

AdcChannelController reservoirTemperatureAdc;
MovingAverageFilter<10> reservoirTemperatureFilter(20);
//NTCSensor reference;

void setMode(MODE mode) {
	osMutexAcquire(s_modeMutex, osWaitForever);
	s_nextMode = mode;
	osMutexRelease(s_modeMutex);
}

float readReservoirTemperatureSensor() {
	uint16_t avalue = reservoirTemperatureAdc.get();
	float r_ntc = RESERVOIR_NTC_INTERNAL_RESISTOR / ((4095.0 / avalue) - 1);

	float temperature = (1.0 / (
			(
					std::log(r_ntc / RESERVOIR_NTC_NOMINAL_RESISTANCE)
					/ RESERVOIR_NTC_BETA)
					+ (1.0 / RESERVOIR_NTC_NOMINAL_TEMPERATURE))) - 273.15;


	if(!isnanf(temperature) && !isinff(temperature)){
		reservoirTemperatureFilter.addValue(temperature);
	}
	return reservoirTemperatureFilter.get();
}

void init() {
	reservoirTemperatureAdc = AdcChannelController(RESERVOIR_NTC_ADC_MODULE,
			RESERVOIR_NTC_ADC_RANK);
	for (size_t i = 0; i < 10; i++) {
		reservoirTemperatureFilter.addValue(readReservoirTemperatureSensor());
	}
}


void update() {
	// read temperature sensor and update od entry.
	OD_ReservoirTemperature_set(readReservoirTemperatureSensor());

	// cooling state maschine.
	osMutexAcquire(s_modeMutex, osWaitForever);
	s_mode = s_nextMode;
	osMutexRelease(s_modeMutex);

	switch (s_mode) {
	case MODE::ON:
		pdu::enableChannel(COOLING_PUMP_CHANNEL);
		break;
	case MODE::ADAPTIV: {
		float pressure = OD_CoolingPressure_get();
		bool errorPressure = (pressure >= RESERVOIR_PRESSURE_HIGH)
				|| (pressure <= RESERVOIR_PRESSURE_LOW);

		bool requiresCooling = (OD_ReservoirTemperature_get() > RESERVOIR_TEMPERATURE_LOW) || clu::requiresCooling();

		bool activate = (not errorPressure) && requiresCooling;
		if (activate) {
			pdu::enableChannel(COOLING_PUMP_CHANNEL);
		} else {
			pdu::disableChannel(COOLING_PUMP_CHANNEL);
		}
		break;
	}
	case MODE::OFF:
		pdu::disableChannel(COOLING_PUMP_CHANNEL);
		break;
	}

}

}
