/*
 * main_entry.cpp
 *
 *  Created on: May 9, 2023
 *      Author: OfficeLaptop
 */

#include "GPIOWriteController.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "PressureSensor.hpp"
#include "estdio.hpp"
#include "DistanceSensor.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {

	// Initalize digital output 1. (valve).
	GPIOWriteController dout1(DOUT1_GPIO_Port, DOUT1_Pin); //VALVE.

	// Initalize digital output 2. (redundancy).
	GPIOWriteController dout2(DOUT2_GPIO_Port, DOUT2_Pin);

	// Initalize analog input to read pressure from pressure 4.
	PressureSensor pressure(ADC_MODULE2, 3, -0.015);

	// Initalize analog input to read pressure from pressure 3.
	//PressureSensor pressure(ADC_MODULE2, 2, -0.015);

	// Initalize analog input to read pressure from pressure 3.
	//PressureSensor pressure(ADC_MODULE2, 2, -0.015);

	// Initalize analog input to read pressure from pressure 3.
	//PressureSensor pressure(ADC_MODULE2, 1, -0.015);

	// Initalize analog input to read pressure from pressure 3.
	//PressureSensor pressure(ADC_MODULE2, 0, -0.015);

	// Initalize analog input to read distance sensor from pressure 4.
	//DistanceSensor distanceSensor(ADC_MODULE2, 3);

	// Initalize analog input to read distance sensor from pressure 3.
	//DistanceSensor distanceSensor(ADC_MODULE2, 2);

	// Initalize analog input to read distance sensor from pressure 2.
	//DistanceSensor distanceSensor(ADC_MODULE2, 1);

	// Initalize analog input to read distance sensor from pressure 1.
	DistanceSensor distanceSensor(ADC_MODULE2, 0);

	while (true) {
		// read pressure in bar 0 ^= atmospheric pressure.
		float bar = pressure.get(true);
		printf("pressure = %f\n", bar);

		// read distance sensor.
		float distance = distanceSensor.get(true);

		printf("distance = %f\n", distance);

		// set digital pin to 12V.
		dout1.set();

		// wait for one second.
		osDelay(1000);

		//set digital pin to GND.
		dout1.reset();

		// wait for one second.
		osDelay(1000);
	}

}

#ifdef __cplusplus
}
#endif
