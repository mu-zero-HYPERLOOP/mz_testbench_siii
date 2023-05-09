/*
 * PodStartLevitation.cpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#include "PodStartLevitationState.hpp"
#include "estdio.hpp"
#include "MergedMdbState.hpp"
#include "GlobalState.hpp"
#include "PodLevitationState.hpp"


void PodStartLevitation::setup(){
	printf("enter start levitation\n");
}

void PodStartLevitation::update(){
	if(MergedMdbState::getInstance().getState() == MDB_STATE_LEVITATION){
		GlobalState::getInstance().setState<PodLevitationState>();
	}
}

void PodStartLevitation::dispose(){
	printf("exit start levitation\n");

}
