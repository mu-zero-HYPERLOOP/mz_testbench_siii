/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "cooling_pressure.hpp"
#include "brake_solenoid.hpp"
#include "state_maschine.hpp"
#include "sensor_ecu_remote.hpp"
#include "proc_info.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	proc_info::init();
	cooling_pressure::init();
	state_maschine::init();
	sensor_ecu::init();
	brake_solenoid::init();

	while(true){
		proc_info::update();
		cooling_pressure::update();
		sensor_ecu::update();
		state_maschine::update();
		osDelay(pdMS_TO_TICKS(50));
	}
}

#ifdef __cplusplus
}
#endif
