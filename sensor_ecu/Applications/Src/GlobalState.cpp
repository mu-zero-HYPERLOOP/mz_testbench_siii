/*
 * GlobalState.cpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */


#include "GlobalState.hpp"
#include "canzero.hpp"

void canzero::handle_emergency_warning(){
	//TODO prevent overwriting of emergency states.
	GlobalState::getInstance().setState<PodEmergencyState>();
}


void canzero::handle_heartbeat_miss(){
	//TODO remove me later
	//GlobalState::getInstance().setState<PodEmergencyState>();
}

void canzero::handle_txmailbox_overflow(CAN_HandleTypeDef* hcan){
	//ignore for now.
	//TODO missing default implementation.
}

void canzero::handle_trasmission_request_error(){
	//ignore for now.
	//TODO missing default implementation.
}

