/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "proc_info.hpp"
#include "sensor_ecu_remote.hpp"
#include "mdb_remote.hpp"
#include "cooling_controll.hpp"
#include "central_controll.hpp"
#include "status_led_controll.hpp"
#include "state_maschine.hpp"
#include "error_prop.hpp"

#ifdef __cplusplus
extern "C" {
#endif


void main_entry(void *argv) {
	info::init();
	error_prop::init();
	cooling_controll::init();
	mdb::init();
	sensor_ecu_remote::init();
	central_controll::init();
	status_led::init();
	state_maschine::init();


	while(true){
		info::update();
		mdb::update();
		sensor_ecu_remote::update();
		cooling_controll::update();
		central_controll::update();
		status_led::update();
		state_maschine::update();
		error_prop::update();

		osDelay(pdMS_TO_TICKS(50));
	}
}

#ifdef __cplusplus
}
#endif
