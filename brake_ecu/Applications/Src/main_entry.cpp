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


#ifdef __cplusplus
extern "C" {
#endif


void main_entry(void *argv) {
	PressureSensor pressureSensor1(ADC_MODULE2, 0, 150, -0.015);
	PressureSensor pressureSensor2(ADC_MODULE2, 1, 150, -0.015);
	PressureSensor pressureSensor3(ADC_MODULE2, 2, 150, -0.015);
	PressureSensor pressureSensor4(ADC_MODULE2, 3, 150, -0.015);

	MovingAverageFilter<32> pressureFilter;

	while(true){
		float p1 = pressureSensor1.get();
		float p2 = pressureSensor2.get();
		float p3 = pressureSensor3.get();
		float p4 = pressureSensor4.get();

		pressureFilter.push(p1);
		pressureFilter.push(p2);
		pressureFilter.push(p3);
		pressureFilter.push(p4);

		float pressure = pressureFilter.average();

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
