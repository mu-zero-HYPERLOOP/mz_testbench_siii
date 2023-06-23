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

static GPIOExtiController extiGpio(DIN2_GPIO_Port, DIN2_Pin);

static volatile uint32_t interruptsCounter = 0;

static constexpr bool FREQUENT_LOGGING = false;

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
	if(FREQUENT_LOGGING){
		can::Message<can::messages::SensorF_SDO_Resp> counterMsg;
		counterMsg.set<can::signals::SensorF_OD_StripeCount>(interruptsCounter);
		counterMsg.send();
	}
}


}
