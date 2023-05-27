/*
 * kistler_controll.hpp
 *
 *  Created on: May 18, 2023
 *      Author: OfficeLaptop
 */

#include <kistler_remote.hpp>
#include "canzero.hpp"

namespace kistler {

void mainDataReceiver(RxMessage &raw) {
	can::Message < can::messages::OpticalSensor_TX_MainData > msg { raw };
	OD_Velocity_set(msg.get<can::signals::OpticalSensor_TX_Vel>());
	OD_Position_set(msg.get<can::signals::OpticalSensor_TX_Distance>());
}

void statusReceiver(RxMessage& raw){
	can::Message<can::messages::OpticalSensor_TX_Status> msg {raw};
	msg.get<can::signals::OpticalSensor_TX_Temp>();
}

void init() {
	can::registerMessageReceiver<can::messages::OpticalSensor_TX_MainData>(mainDataReceiver);
	can::registerMessageReceiver<can::messages::OpticalSensor_TX_Status>(statusReceiver);
}


void enable(){

}

void disable(){

}

void update() {
}

}
