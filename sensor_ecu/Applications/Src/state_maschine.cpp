/*
 * state_maschine.cpp
 *
 *  Created on: May 11, 2023
 *      Author: OfficeLaptop
 */

#include "state_maschine.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"


namespace state_maschine {

PodState s_state;
PodState s_lastState;
PodState s_nextState;
osMutexId_t s_stateMutex = osMutexNew(NULL);

void setState(PodState state){
    osMutexAcquire(s_stateMutex, portMAX_DELAY);
	s_nextState = state;
    osMutexRelease(s_stateMutex);
}

void update(){
    osMutexAcquire(s_stateMutex, portMAX_DELAY);
	s_lastState = s_state;
	s_state = s_nextState;
    osMutexRelease(s_stateMutex);

    switch(s_state){
    case STATE::POD_IDLE:
    	break;
    case STATE::POD_LAUNCH_PREPARATION:
    	break;
    case STATE::POD_READY_TO_LAUNCH:
    	break;
    case STATE::POD_START_LEVITATION:
    	break;
    case STATE::POD_LEVITATING:
    	break;
    case STATE::POD_LAUNCHING:
    	break;
    case STATE::POD_STOP_LEVITATION:
    	break;
    case STATE::POD_BREAKING:
    	break;
    case STATE::POD_STOP:
    	break;
    case STATE::POD_SAFE_TO_APPROACH:
    	break;
    case STATE::POD_FAULT:
    	break;
    }
}

void start(){
	s_state = s_nextState;
	s_lastState = s_nextState;
	while(true){
		update();
		can::Message<can::messages::SensorF_TX_StatePod> stateMsg;
		stateMsg.set<can::signals::SensorF_TX_PodState>(s_state);
		stateMsg.set<can::signals::SensorF_TX_PodState_Target>(s_nextState);
		stateMsg.set<can::signals::SensorF_TX_PodState_Last>(s_lastState);
		stateMsg.send();
	}
}

}


