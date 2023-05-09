/*
 * PodEmergencyState.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include <PodEmergencyState.hpp>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "PodIdleState.hpp"
#include "PodStartupState.hpp"
#include "StateMaschine.hpp"
#include "PodLaunchPreparationState.hpp"
#include "PodLaunchingState.hpp"



void PodEmergencyState::setup() {
	printf("enter emcy state\n");
}

void PodEmergencyState::update() {
	if(m_stateMaschine->wasPreviousState<PodIdleState>()){
		osDelay(osWaitForever);
	}else if(m_stateMaschine->wasPreviousState<PodStartupState>()){
		osDelay(osWaitForever);
	}else if(m_stateMaschine->wasPreviousState<PodLaunchPreparationState>()){
		//TODO implement handling
		Error_Handler();
		osDelay(osWaitForever);
	}else if(m_stateMaschine->wasPreviousState<PodLaunchingState>()){
		//TODO implement handing.
		Error_Handler();
		osDelay(osWaitForever);
	}
	osDelay(50);
}

void PodEmergencyState::dispose() {
	printf("exit emcy state\n");
}
