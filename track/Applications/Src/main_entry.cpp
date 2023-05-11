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


#ifdef __cplusplus
extern "C" {
#endif


void main_entry(void *argv) {
	GPIOWriteController piston(VALVE_GPIO_Port, VALVE_Pin);
	PressureSensor pressure(ADC_MODULE2, 0);
	piston.set();
	while(true){
		float bar = pressure.get();
		printf("pressure = %f\n",bar);
		osDelay(1000);
	}

}

#ifdef __cplusplus
}
#endif
