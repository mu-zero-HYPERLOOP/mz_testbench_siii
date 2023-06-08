/*
 * state_maschine.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "state_maschine.hpp"
#include "sensor_ecu_remote.hpp"
#include "brake_solenoid.hpp"
#include "sdc.hpp"
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
    case STATE::POD_STARTUP:
    case STATE::POD_IDLE:
    	sdc::open();
    	brake_solenoid::engage();
    	break;
    case STATE::POD_LAUNCH_PREPARATION:
    case STATE::POD_READY_TO_LAUNCH:
    case STATE::POD_START_LEVITATION:
    case STATE::POD_STABLE_LEVITATION:
    case STATE::POD_CRUSING:
    	brake_solenoid::engage();
    	sdc::close();
    	break;
    case STATE::POD_DISENGAGE_BRAKES:
    case STATE::POD_STOP_LEVITATION:
    	sdc::close();
    	brake_solenoid::disengage();
    	break;
    case STATE::POD_ROLLING:
    	sdc::open();
    	brake_solenoid::disengage();
    	break;
    case STATE::POD_ENGAGE_BRAKES:
    case STATE::POD_END_OF_RUN:
    case STATE::POD_SAFE_TO_APPROCH:
    	sdc::open();
    	brake_solenoid::engage();
    	break;
    case STATE::POD_PUSHABLE:
    	sdc::open();
    	brake_solenoid::disengage();
    	break;
    }
}

}
