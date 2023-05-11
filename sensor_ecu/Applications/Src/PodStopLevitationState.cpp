/*
 * PodRunStopState.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include <PodStopLevitationState.hpp>
#include "estdio.hpp"
#include "SDC.hpp"
#include "MergedMdbState.hpp"
#include "GlobalState.hpp"
#include "PodBreakState.hpp"

void PodStopLevitationState::setup() {
	printf("enter stop levitation\n");
	SDC::getInstance().open();
}

void PodStopLevitationState::update() {
	if(MergedMdbState::getInstance().getState() == MDB_STATE_GROUNDED){
		GlobalState::getInstance().setState<PodBreakState>();
	}
}

void PodStopLevitationState::dispose() {
	printf("exit stop levitation\n");
}
