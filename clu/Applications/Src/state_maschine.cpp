/*
 * state_maschine.hpp
 *
 *  Created on: May 27, 2023
 *      Author: OfficeLaptop
 */

#include "state_maschine.hpp"
#include "sensor_ecu_remote.hpp"
#include "State.hpp"
#include "cooling_controll.hpp"

namespace state_maschine {

void init(){


}

void update(){
	PodState state = sensor_ecu_remote::getState();
    switch(s_state){
    case STATE::POD_STARTUP:
    case STATE::POD_IDLE:
    case STATE::POD_LAUNCH_PREPARATION:
    case STATE::POD_READY_TO_LAUNCH:
    case STATE::POD_START_LEVITATION:
    case STATE::POD_STABLE_LEVITATION:
    case STATE::POD_CRUSING:
    case STATE::POD_DISENGAGE_BRAKES:
    case STATE::POD_STOP_LEVITATION:
    case STATE::POD_ROLLING:
    case STATE::POD_ENGAGE_BRAKES:
    case STATE::POD_END_OF_RUN:
    case STATE::POD_SAFE_TO_APPROCH:
    case STATE::POD_PUSHABLE:
    case STATE::POD_OFF:
    	cooling_controll::setCoolingThreshold(40);
    	break;
    }
}

}
