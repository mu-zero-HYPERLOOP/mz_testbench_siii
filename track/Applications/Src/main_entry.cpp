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
	GPIOWriteController piston(DOUT2_GPIO_Port, DOUT2_Pin);
	GPIOWriteController pistonOpt(VALVE_GPIO_Port, VALVE_Pin);
	PressureSensor pressure(ADC_MODULE2, 3, -0.015);
	piston.set();
	while(true){
		float bar = pressure.get(true);
		printf("pressure = %f\n",bar);
		piston.toggle();
		pistonOpt.toggle();

		osDelay(1000);
	}

}

#ifdef __cplusplus
}
#endif
