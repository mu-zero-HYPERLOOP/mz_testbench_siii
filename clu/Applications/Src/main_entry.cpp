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
	SDC& sdc = GlobalPeripheralRegistry::getInstance().getSDC();
	ExternalMdbDistanceSensor* mdbDistanceSensor = GlobalPeripheralRegistry::getInstance().getMdbDistanceSensors();
	size_t mdbCount = GlobalPeripheralRegistry::getInstance().getMdbDistanceSensorCount();
	while(true){

		//TODO perform plane fitting and central controll here.

		osDelay(pdMS_TO_TICKS(50));
	}
}

#ifdef __cplusplus
}
#endif
