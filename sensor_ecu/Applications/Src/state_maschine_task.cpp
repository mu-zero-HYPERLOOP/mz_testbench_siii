/*
 * state_maschine_entry.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */



#include "StateMaschine.hpp"
#include "PodEmergencyState.hpp"
#include "PodIdleState.hpp"
#include "PodLaunchingState.hpp"
#include "PodLaunchPreparationState.hpp"
#include "PodPushableState.h"
#include "PodReadyToLaunchState.hpp"
#include "PodRunStopState.hpp"
#include "PodSafeToApproach.hpp"
#include "PodStartupState.hpp"


#ifdef __cplusplus
extern "C" {
#endif


void state_maschine_entry(void *argv) {
	StateMaschineMemory<8> fmsMemory;
	StateMaschine fms(&fmsMemory);

	PodEmergencyState emergenyState;
	fms.registerState(emergenyState);

	PodIdleState idleState;
	fms.registerState(idleState);

	PodLaunchingState launchingState;
	fms.registerState(launchingState);

	PodLaunchPreparationState launchPrepState;
	fms.registerState(launchPrepState);

	PodPushableState pushableState;
	fms.registerState(pushableState);

	PodReadyToLaunchState readyToLaunchState;
	fms.registerState(readyToLaunchState);

	PodRunStopState runStopState;
	fms.registerState(runStopState);

	PodSafeToApproach safeToApprochState;
	fms.registerState(safeToApprochState);

	PodStartupState startupState;
	fms.registerState(startupState);

	fms.start<PodStartupState>();
}

#ifdef __cplusplus
}
#endif
