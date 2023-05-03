/*
 * PodReadyToLaunchState.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include <PodReadyToLaunchState.hpp>
#include "StateMaschine.hpp"
#include "peripheral_config.hpp"

PodReadyToLaunchState::PodReadyToLaunchState() :
	m_coolingTemperatur(g_peripherals.m_coolingReservoirTemperaturSensorConfig),
	m_eboxTemperatur(g_peripherals.m_eboxTemperaturConfig){
	// TODO Auto-generated constructor stub

}


void PodReadyToLaunchState::setup() {

}

void PodReadyToLaunchState::update() {

}

void PodReadyToLaunchState::dispose() {

}
