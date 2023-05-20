/*
 * sensor_ecu_remote.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "sensor_ecu_remote.hpp"
#include "canzero.hpp"
#include <cinttypes>
#include "State.hpp"


namespace sensor_ecu {

static volatile PodState state = STATE::POD_OFF;

void statusReceiver(RxMessage& raw){
	can::Message<can::messages::SensorF_TX_StatePod> msg{raw};
	state = msg.get<can::signals::SensorF_TX_PodState>();
}

void init(){
	can::registerMessageReceiver<can::messages::SensorF_TX_StatePod>(statusReceiver);
}

void update(){
	printf("cooling pressure = %f\n", OD_CoolingPressure_get());
	can::Message<can::messages::BrakeF_TX_PressureCooling> coolingMsg;
	coolingMsg.set<can::signals::BrakeF_TX_Pressure_Reservoir>(OD_CoolingPressure_get());
	coolingMsg.send();
}

PodState getState(){
	return state;
}

}
