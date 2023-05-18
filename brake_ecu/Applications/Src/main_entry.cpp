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
	PressureSensor pressureSensor1(ADC_MODULE2, 0, 150, 0);
	PressureSensor pressureSensor2(ADC_MODULE2, 1, 150, 0);
	PressureSensor pressureSensor3(ADC_MODULE2, 2, 150, 0);
	PressureSensor pressureSensor4(ADC_MODULE2, 3, 150, -0.015);
	MovingAverageFilter<32> pressureFilter;

	while(true){
		float p4 = pressureSensor4.get(true);

		pressureFilter.push(p4);

		float pressure = p4;

		OD_Pressure1_set(pressure);
		can::Message<can::messages::BrakeF_TX_Pressure> pressureMsg;
		pressureMsg.set<can::signals::BrakeF_TX_Pressure_Act>(0);
		pressureMsg.set<can::signals::BrakeF_TX_Pressure_Retract>(0);
		pressureMsg.set<can::signals::BrakeF_TX_Pressure_Tank>(pressure);
		pressureMsg.send();

		printf("pressure = %f\n", pressure);

		osDelay(pdMS_TO_TICKS(1000));
	}
}

#ifdef __cplusplus
}
#endif
