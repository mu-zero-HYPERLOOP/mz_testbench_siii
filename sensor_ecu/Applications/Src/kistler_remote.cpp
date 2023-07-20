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

static uint32_t valueCounter;
static volatile float vel;
static volatile float pos;
static float offset;

static constexpr bool FREQUENT_LOGGING = true;

pdu::HpChannel POWER_CHANNEL = pdu::HpChannel::HP_CHANNEL4;

void mainDataReceiver(RxMessage &raw) {
	taskENTER_CRITICAL();
	can::Message < can::messages::OpticalSensor_TX_MainData > msg { raw };
	vel += msg.get<can::signals::OpticalSensor_TX_Vel>();
	pos += msg.get<can::signals::OpticalSensor_TX_Distance>();
	valueCounter++;
	taskEXIT_CRITICAL();
}

void statusReceiver(RxMessage& raw){
	can::Message<can::messages::OpticalSensor_TX_Status> msg {raw};
	msg.get<can::signals::OpticalSensor_TX_Temp>();
}

void init() {
	can::registerMessageReceiver<can::messages::OpticalSensor_TX_MainData>(mainDataReceiver);
	can::registerMessageReceiver<can::messages::OpticalSensor_TX_Status>(statusReceiver);
	enable();
	offset = 0;
}


void enable(){
	pdu::enableChannel(POWER_CHANNEL);
}

void disable(){
	pdu::disableChannel(POWER_CHANNEL);
}

void normalize(){
	offset = OD_Position_get();
	OD_Position_set(pos - offset);
}

void update() {
	taskENTER_CRITICAL();
	if(valueCounter > 0.0){
		OD_Velocity_set(vel / valueCounter);
		OD_Position_set((pos / valueCounter) - offset);
	}
	taskEXIT_CRITICAL();
	if(FREQUENT_LOGGING){
		can::Message<can::messages::SensorF_SDO_Resp> velMsg;
		velMsg.set<can::signals::SensorF_SDO_ID>(can::signals::SensorF_SDO_ID::VELOCITY);
		velMsg.set<can::signals::SensorF_OD_Velocity>(vel);
		velMsg.send();
		can::Message<can::messages::SensorF_SDO_Resp> posMsg;
		posMsg.set<can::signals::SensorF_SDO_ID>(can::signals::SensorF_SDO_ID::POSITION);
		posMsg.set<can::signals::SensorF_OD_Position>(pos);
		posMsg.send();
	}
}

}
