/*
 * PodStartLevitation.cpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#include <PodLevitationState.hpp>
#include "estdio.hpp"
#include "MergedMdbState.hpp"
#include "GlobalState.hpp"
#include "GroundStationReceiver.hpp"
#include "PodLaunchingState.hpp"

void PodLevitationState::setup(){
	printf("enter levitation\n");
}

void PodLevitationState::update(){
	if(GroundStationReceiver::getInstance().getLastCommand() == COMMAND_LAUNCH){
		GlobalState::getInstance().setState<PodLaunchingState>();
	}
}

void PodLevitationState::dispose(){
	printf("exit levitation\n");

}
