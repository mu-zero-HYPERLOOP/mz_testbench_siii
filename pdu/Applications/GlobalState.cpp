/*
 * GlobalState.cpp
 *
 *  Created on: Jun 18, 2023
 *      Author: OfficeLaptop
 */
#include "GlobalState.hpp"

extern MessageBufferHandle_t handlePodStateMessageBuffer;

namespace state{

static PodState state = STATE::POD_STARTUP;

PodState get(){
	return state;
}

//TODO handle SECU Heartbeat miss.

void receiveCAN(){
	RxMessage rxMsgRaw;
	 while(xMessageBufferReceive(handlePodStateMessageBuffer, &rxMsgRaw, sizeof(rxMsgRaw), 0) != 0) {
		 can::Message<can::messages::SensorF_TX_StatePod> msg{rxMsgRaw};
		 state = msg.get<can::signals::SensorF_TX_PodState>();
	 }
}

}


