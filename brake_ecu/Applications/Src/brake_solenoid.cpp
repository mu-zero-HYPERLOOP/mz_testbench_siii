/*
 * brake_solenoid.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "brake_solenoid.hpp"
#include "GPIOWriteController.hpp"
#include "gpio.h"
#include "canzero.hpp"
#include <cinttypes>

namespace brake_solenoid {

GPIOWriteController solenoidGpio(DOUT1_GPIO_Port, DOUT1_Pin);


void init(){
	solenoidGpio.set();
	OD_BrakeStatus_set(static_cast<uint8_t>(BrakeStatus::Disengage));
}

void engage(){
	solenoidGpio.reset();
	OD_BrakeStatus_set(static_cast<uint8_t>(BrakeStatus::Engage));
}

void disengage(){
	solenoidGpio.set();
	OD_BrakeStatus_set(static_cast<uint8_t>(BrakeStatus::Disengage));
}


}
