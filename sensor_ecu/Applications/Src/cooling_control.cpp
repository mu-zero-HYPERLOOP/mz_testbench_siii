/*
 * cooling_control.hpp
 *
 *  Created on: 11 May 2023
 *      Author: karl
 */


#include <brake_ecu_remote.hpp>
#include <pdu_remote.hpp>
#include "cooling_controll.hpp"
#include "NTCSensor.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "canzero.hpp"
#include "AdcModule.hpp"
#include "AdcChannelController.hpp"

namespace cooling {

constexpr float THRESHOLD = 30;

static MODE s_mode;
static MODE s_nextMode = MODE::ADAPTIV;
static osMutexId_t s_modeMutex = osMutexNew(NULL);

constexpr float RESERVOIR_PRESSURE_HIGH = 5;
constexpr float RESERVOIR_PRESSURE_LOW = 2;

constexpr float RESERVOIR_TEMPERATURE_LOW = 20;
constexpr float MAGNET_TEMPERATURE_LOW = 30;

constexpr float     RESERVOIR_NTC_R25               = 10000;
constexpr float     RESERVOIR_NTC_BETA              = 3950;
constexpr float     RESERVOIR_NTC_INTERNAL_RESISTOR = 100000;
constexpr float     RESERVOIR_NTC_INTERNAL_SUPPLY   = 3.3;
constexpr AdcModule RESERVOIR_NTC_ADC_MODULE        = ADC_MODULE2;
constexpr uint16_t  RESERVOIR_NTC_ADC_RANK          = 0;

pdu::HpChannel COOLING_PUMP_CHANNEL = pdu::HP_CHANNEL1;


NTCSensor reservoirTemperatureSensor;

void setMode(MODE mode) {
	osMutexAcquire(s_modeMutex, osWaitForever);
	s_nextMode = mode;
	osMutexRelease(s_modeMutex);
}

bool toggle = true;

void init(){
	reservoirTemperatureSensor = NTCSensor( RESERVOIR_NTC_ADC_MODULE,
											RESERVOIR_NTC_ADC_RANK,
											RESERVOIR_NTC_R25,
											RESERVOIR_NTC_BETA,
											RESERVOIR_NTC_INTERNAL_RESISTOR,
											RESERVOIR_NTC_INTERNAL_SUPPLY);
}


void update(){
	// reading temperature sensor.
	float reservoirTemp = reservoirTemperatureSensor.getTemperaturC();

	// filter temperature sensor.
	float filteredReservoirTemp = reservoirTemp;

	printf("reservoir temp = %f\n", reservoirTemp);

	// update od entry.
	OD_ReservoirTemperature_set(filteredReservoirTemp);

	// cooling state maschine.
	osMutexAcquire(s_modeMutex, osWaitForever);
	s_mode = s_nextMode;
	osMutexRelease(s_modeMutex);

	switch(s_mode){
	case MODE::ON:
		pdu::enableChannel(COOLING_PUMP_CHANNEL);
		break;
	case MODE::ADAPTIV:
	{
		float pressure = OD_CoolingPressure_get();
		bool errorPressure = (pressure >= RESERVOIR_PRESSURE_HIGH) || (pressure <= RESERVOIR_PRESSURE_LOW);

		bool requiresCooling = (OD_ReservoirTemperature_get() > RESERVOIR_TEMPERATURE_LOW) ||
				(OD_Magnet_1_Temperature_get() > MAGNET_TEMPERATURE_LOW) ||
				(OD_Magnet_2_Temperature_get() > MAGNET_TEMPERATURE_LOW) ||
				(OD_Magnet_3_Temperature_get() > MAGNET_TEMPERATURE_LOW) ||
				(OD_Magnet_4_Temperature_get() > MAGNET_TEMPERATURE_LOW) ||
				(OD_Magnet_5_Temperature_get() > MAGNET_TEMPERATURE_LOW) ||
				(OD_Magnet_6_Temperature_get() > MAGNET_TEMPERATURE_LOW);

		bool activate = (not errorPressure) && requiresCooling;
		if(activate){
			pdu::enableChannel(COOLING_PUMP_CHANNEL);
		}else{
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
