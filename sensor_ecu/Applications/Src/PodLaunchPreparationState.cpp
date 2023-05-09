/*
 * PodLaunchPreparationState.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include <BrakeECUController.hpp>
#include <PodLaunchPreparationState.hpp>
#include "GlobalState.hpp"
#include "peripheral_config.hpp"
#include "FreeRTOS.h"
#include "PodStartLevitationState.hpp"
#include "cmsis_os.h"
#include "estdio.hpp"
#include "MergedMdbState.hpp"
#include "PDUController.hpp"

void PodLaunchPreparationState::setup() {
	printf("enter launch prep\n");
	PDUController::getInstance().enable();
	PDUController::getInstance().enableHV();
	BrakeECUController::getInstance().disengageBrakes();
	SDC::getInstance().close();
}

void PodLaunchPreparationState::update() {
	if (MergedMdbState::getInstance().getState() == MDB_STATE_PRECHARGE_DONE
			&& PDUController::getInstance().isEnabled()
			&& PDUController::getInstance().isHVEnabled()
			&& not PDUController::getInstance().hasError()
			&& BrakeECUController::getInstance().getBrakeState() == BRAKE_DISENGAGED) {
		GlobalState::getInstance().setState<PodReadyToLaunchState>();
	}
	osDelay(50);
}

void PodLaunchPreparationState::dispose() {
	printf("exit launch prep\n");
}
