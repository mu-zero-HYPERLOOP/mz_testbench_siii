/*
 * PodIdleState.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include "PodIdleState.hpp"
#include "PodLaunchPreparationState.hpp"
#include "StateMaschine.hpp"
#include "GlobalState.hpp"
#include "GroundStationReceiver.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"

void PodIdleState::setup(){
	printf("enter idle state\n");
}

void PodIdleState::update(){
	//TODO check that all ecus are connected using either a improved heartbeat or a custom method.
	if(GroundStationReceiver::getInstance().getLastCommand() == COMMAND_ENTER_LAUNCH_PREP){
		GlobalState::getInstance().setState<PodLaunchPreparationState>();
	}
	osDelay(50);
}

void PodIdleState::dispose(){
	printf("exit idle state\n");
}

