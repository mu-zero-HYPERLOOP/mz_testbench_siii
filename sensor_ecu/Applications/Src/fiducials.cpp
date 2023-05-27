/*
 * fiducials.hpp
 *
 *  Created on: May 20, 2023
 *      Author: OfficeLaptop
 */

#include "fiducials.hpp"
#include "GPIOExtiController.hpp"
#include "gpio.h"
#include <cinttypes>
#include <canzero.hpp>

namespace fiducials {

static GPIOExtiController extiGpio(DIN1_GPIO_Port, DIN1_Pin);

static volatile uint32_t interruptsCounter = 0;

void extiCallback(bool edge){
	interruptsCounter += 1;
}

void reset(){
	interruptsCounter = 0;
}

void init(){
	extiGpio.setExtiCallback(extiCallback);
}

void update(){
	OD_StripeCount_set(interruptsCounter);
}


}
