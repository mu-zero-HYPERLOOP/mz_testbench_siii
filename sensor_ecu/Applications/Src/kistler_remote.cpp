/*
 * kistler_controll.hpp
 *
 *  Created on: May 18, 2023
 *      Author: OfficeLaptop
 */

#include <kistler_remote.hpp>
#include "canzero.hpp"
#include "pdu_remote.hpp"

namespace kistler {

static volatile float vel;
static volatile float pos;

static constexpr bool FREQUENT_LOGGING = true;

pdu::HpChannel POWER_CHANNEL = pdu::HpChannel::HP_CHANNEL4;

void mainDataReceiver(RxMessage &raw) {
	can::Message < can::messages::OpticalSensor_TX_MainData > msg { raw };
	vel = msg.get<can::signals::OpticalSensor_TX_Vel>();
	pos = msg.get<can::signals::OpticalSensor_TX_Distance>();
}

void statusReceiver(RxMessage& raw){
	can::Message<can::messages::OpticalSensor_TX_Status> msg {raw};
	msg.get<can::signals::OpticalSensor_TX_Temp>();
}

void init() {
	can::registerMessageReceiver<can::messages::OpticalSensor_TX_MainData>(mainDataReceiver);
	can::registerMessageReceiver<can::messages::OpticalSensor_TX_Status>(statusReceiver);
	enable();
}


void enable(){
	pdu::enableChannel(POWER_CHANNEL);
}

void disable(){
	pdu::disableChannel(POWER_CHANNEL);
	OD_Position_set(0);
	OD_Velocity_set(0);
}

void update() {
	OD_Velocity_set(vel);
	OD_Position_set(pos);
	if(FREQUENT_LOGGING){
		can::Message<can::messages::SensorF_SDO_Resp> velMsg;
		velMsg.set<can::signals::SensorF_OD_Velocity>(vel);
		velMsg.send();
		can::Message<can::messages::SensorF_SDO_Resp> posMsg;
		posMsg.set<can::signals::SensorF_OD_Position>(pos);
		posMsg.send();
	}
}

}
