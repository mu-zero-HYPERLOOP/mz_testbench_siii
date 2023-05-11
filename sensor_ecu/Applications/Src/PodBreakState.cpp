/*
 * PodBreakState.cpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#include "PodBreakState.hpp"
#include "BrakeECUController.hpp"
#include "EstimatedStateRegistry.hpp"

#include "estdio.hpp"

void PodBreakState::setup() {
	printf("enter break state\n");
	BrakeECUController::getInstance().engageBrakes();
}

void PodBreakState::update() {
	osDelay(pdMS_TO_TICKS(50));

}

void PodBreakState::dispose() {
	printf("exit break state\n");
}
