/*
 * cooling_control.hpp
 *
 *  Created on: 11 May 2023
 *      Author: karl
 */


#include "cooling_controll.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "pdu_control.hpp"
#include "brake_ecu_controll.hpp"
#include "canzero.hpp"

namespace cooling {

constexpr float THRESHOLD = 30;

static MODE s_mode;
static MODE s_nextMode = MODE::ADAPTIV;
static osMutexId_t s_modeMutex = osMutexNew(NULL);

constexpr float RESERVOIR_PRESSURE_HIGH = 5;
constexpr float RESERVOIR_PRESSURE_LOW = 2;

constexpr float RESERVOIR_TEMPERATURE_LOW = 20;
constexpr float MAGNET_TEMPERATURE_LOW = 30;


pdu::HpChannel COOLING_PUMP_CHANNEL = pdu::HP_CHANNEL1;

void setMode(MODE mode) {
	osMutexAcquire(s_modeMutex, osWaitForever);
	s_nextMode = mode;
	osMutexRelease(s_modeMutex);
}

bool toggle = true;

void update(){
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
	}
	case MODE::OFF:
		pdu::disableChannel(COOLING_PUMP_CHANNEL);
		break;
	}

}

}
