/*
 * state_maschine.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "state_maschine.hpp"
#include "sensor_ecu_remote.hpp"
#include "brake_solenoid.hpp"
#include "State.hpp"

namespace state_maschine{

void init(){
}

void update(){
	PodState state = sensor_ecu::getState();
	// TODO implement low level control of the brakes.
	// brake_solenoid::engage();
	// brake_solenoid::disengage();
	switch(state){
	case STATE::POD_OFF:
	case STATE::POD_IDLE:
	case STATE::POD_LAUNCH_PREPARATION:
	case STATE::POD_FAULT:
	case STATE::POD_READY_TO_LAUNCH:
	case STATE::POD_LAUNCHING:
	case STATE::POD_PUSHABLE:
	case STATE::POD_SAFE_TO_APPROACH:
	case STATE::POD_START_LEVITATION:
	case STATE::POD_STOP_LEVITATION:
	case STATE::POD_LEVITATING:
	case STATE::POD_BREAKING:
	case STATE::POD_STOP:
	default:
		break;
	}
}

}
