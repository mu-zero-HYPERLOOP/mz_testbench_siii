/*
 * PodLaunchPreparationState.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include <PodLaunchPreparationState.hpp>
#include "peripheral_config.hpp"

PodLaunchPreparationState::PodLaunchPreparationState() :
		m_coolingPressure(g_peripherals.m_pressureConfig.m_module, g_peripherals.m_pressureConfig.m_rank),
		m_coolingTemperatur( g_peripherals.m_coolingReservoirTemperaturSensorConfig),
		m_sdc(g_peripherals.m_sdcConfig) {
}

void PodLaunchPreparationState::setup() {

}

void PodLaunchPreparationState::update() {
	//TODO implement sensor value checking
	//TODO implement checking of received messages
}

void PodLaunchPreparationState::dispose() {

}
