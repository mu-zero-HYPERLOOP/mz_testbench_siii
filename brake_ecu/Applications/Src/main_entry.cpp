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
	SolenoidController& solenoid = GlobalPeripheralRegistry::getInstance().getSolenoidController();
	SDC& sdc = GlobalPeripheralRegistry::getInstance().getSDC();
	while(true){
		//solenoid.activate();
		//solenoid.deactivate();
		//sdc.open();
		//sdc.close();
		osDelay(pdMS_TO_TICKS(50));
	}
}

#ifdef __cplusplus
}
#endif
