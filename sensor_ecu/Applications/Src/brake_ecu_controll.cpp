/*
 * brake_controll.cpp
 *
 *  Created on: May 14, 2023
 *      Author: OfficeLaptop
 */


#include <brake_ecu_remote.hpp>
#include "canzero.hpp"


namespace brake {

void pressureReceiver(RxMessage& raw){
	can::Message<can::messages::BrakeF_TX_PressureCooling> msg{raw};
	OD_CoolingPressure_set(msg.get<can::signals::BrakeF_TX_Pressure_Reservoir>());
	printf("received pressure = %f\n", OD_CoolingPressure_get());
}

void init(){
	can::registerMessageReceiver<can::messages::BrakeF_TX_PressureCooling>(pressureReceiver);
}


void update(){

}

}
