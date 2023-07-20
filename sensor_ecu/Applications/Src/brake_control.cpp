/*
 * brake_controll.hpp
 *
 *  Created on: Jun 21, 2023
 *      Author: OfficeLaptop
 */
#include "brake_control.hpp"

#include "pdu_remote.hpp"

namespace brake_control {

constexpr pdu::LpChannel BRAKE_CHANNEL = pdu::LpChannel::LP_CHANNEL4;

void init(){
	engage();
}

void engage(){
	pdu::disableChannel(BRAKE_CHANNEL);
}

void disengage(){
	pdu::enableChannel(BRAKE_CHANNEL);
}

void update(){

}

}
