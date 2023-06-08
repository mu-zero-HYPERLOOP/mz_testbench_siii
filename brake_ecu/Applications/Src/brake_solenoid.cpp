/*
 * brake_solenoid.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "estdio.hpp"
#include "brake_solenoid.hpp"
#include "GPIOWriteController.hpp"
#include "gpio.h"
#include "canzero.hpp"
#include <cinttypes>

namespace brake_solenoid {

static GPIOWriteController solenoidGpio(DOUT1_GPIO_Port, DOUT1_Pin);
static bool current;


void init(){
	solenoidGpio.set();
	current = true;
	OD_BrakeStatus_set(static_cast<uint8_t>(BrakeStatus::Disengage));
}

void engage(){
	if(current){
		printf("engage brakes\n");
	}
	solenoidGpio.reset();
	current = false;
	OD_BrakeStatus_set(static_cast<uint8_t>(BrakeStatus::Engage));
}

void disengage(){
	if(not current){
		printf("disengage brakes\n");
	}
	solenoidGpio.set();
	current = true;
	OD_BrakeStatus_set(static_cast<uint8_t>(BrakeStatus::Disengage));
}


}
