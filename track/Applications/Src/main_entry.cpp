/*
 * main_entry.cpp
 *
 *  Created on: May 9, 2023
 *      Author: OfficeLaptop
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "estdio.hpp"
#include "distance_sensor.hpp"
#include "propulsion_pressure.hpp"
#include "ground_station_remote.hpp"
#include "solenoid.hpp"
#include "state_maschine.hpp"
#include "sdc.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	distance_sensor::init();
	propulsion_pressure::init();
	solenoid::init();
	ground_station::init();
	state_maschine::init();
	sdc::init();

	while (true) {
		distance_sensor::update();
		propulsion_pressure::update();
		ground_station::update();
		solenoid::update();
		state_maschine::update();
		sdc::update();

		// wait for one second.
		osDelay(50);
	}

}

#ifdef __cplusplus
}
#endif
