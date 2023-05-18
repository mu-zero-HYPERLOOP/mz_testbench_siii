/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include "GlobalPeripheralRegistry.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "GlobalStateReceiver.hpp"
#include "estdio.hpp"
#include "PressureSensor.hpp"
#include "estdio.hpp"
#include "MovingAverageFilter.hpp"
#include "canzero.hpp"


#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	PressureSensor reservoirPressureSensor(ADC_MODULE2, 3, 150, -0.015);

	while(true){
		float reservoirPressure = reservoirPressureSensor.get(true);

		printf("reservoirPressure = %f\n" , reservoirPressure);

		// SEND CAN MESSAGES

		can::Message<can::messages::BrakeF_TX_PressureCooling> pressureCoolingMsg;
		pressureCoolingMsg.set<can::signals::BrakeF_TX_Pressure_Reservoir>(reservoirPressure);
		pressureCoolingMsg.send();

		osDelay(pdMS_TO_TICKS(50));
	}
}

#ifdef __cplusplus
}
#endif
