/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include "GlobalPeripheralRegistry.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	DistanceSensor &distanceSensor =
			GlobalPeripheralRegistry::getInstance().getDistanceSensor();
	PneumaticPistonController &pistonController =
			GlobalPeripheralRegistry::getInstance().getPneumaticPistonController();
	while (true) {
		//float distance = distanceSensor.read();

		//pistonController.activate();
		//pistonController.deactivate();

		//put logic here.
		osDelay(pdMS_TO_TICKS(50));
	}
}

#ifdef __cplusplus
}
#endif
