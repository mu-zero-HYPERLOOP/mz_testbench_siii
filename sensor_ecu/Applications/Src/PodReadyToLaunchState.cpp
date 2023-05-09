/*
 * PodReadyToLaunchState.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include <PodReadyToLaunchState.hpp>
#include "StateMaschine.hpp"
#include "peripheral_config.hpp"
#include "GlobalState.hpp"
#include "estdio.hpp"
#include "GroundStationReceiver.hpp"


void PodReadyToLaunchState::setup() {
	printf("enter pod ready to launch\n");
}

void PodReadyToLaunchState::update() {
	if(GroundStationReceiver::getInstance().getLastCommand() == COMMAND_LAUNCH){
		GlobalState::getInstance().setState<PodStartLevitation>();
	}
}

void PodReadyToLaunchState::dispose() {
	printf("exit pod ready to launch\n");
}
