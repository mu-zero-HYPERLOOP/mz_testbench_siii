/*
 * ground_station_remote.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "ground_station_remote.hpp"
#include "State.hpp"
#include "canzero.hpp"
#include <cinttypes>

namespace ground_station {


static volatile PodState state;


void stateReceiver(RxMessage& raw){
	can::Message<can::messages::SensorF_TX_StatePod> msg{raw};
	state = msg.get<can::signals::SensorF_TX_PodState>();
}

void init(){
	can::registerMessageReceiver<can::messages::SensorF_TX_StatePod>(stateReceiver);
}

void update(){

}

PodState getState(){
	return state;
}

}
