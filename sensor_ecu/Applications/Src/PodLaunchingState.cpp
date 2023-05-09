/*
 * PodLaunchingState.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include <PodLaunchingState.hpp>
#include "estdio.hpp"
#include "EstimatedStateRegistry.hpp"
#include "GlobalState.hpp"
#include "PodStopLevitationState.hpp"


void PodLaunchingState::setup() {
	printf("enter launching state\n");
}

void PodLaunchingState::update() {
	if(EstimatedStateRegistry::getInstance().getPosition() > 2){
		GlobalState::getInstance().setState<PodStopLevitationState>();
	}
}

void PodLaunchingState::dispose() {
	printf("exit launching state\n");

}
