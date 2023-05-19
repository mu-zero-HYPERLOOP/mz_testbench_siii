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

void init() {
	can::registerMessageReceiver<can::messages::OpticalSensor_TX_MainData>(mainDataReceiver);
}

void update() {

}

}
