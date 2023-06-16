/*
 * sensor_ecu.hpp
 *
 *  Created on: May 20, 2023
 *      Author: OfficeLaptop
 */

#include "sensor_ecu_remote.hpp"
#include "mdb_remote.hpp"
#include "canzero.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

namespace sensor_ecu_remote {

static volatile PodState m_state = STATE::POD_OFF;

PodState getState() {
	return m_state;
}

void stateReceiver(RxMessage &raw) {
	can::Message<can::messages::SensorF_TX_StatePod> msg { raw };
	m_state =
			static_cast<PodState>(msg.get<can::signals::SensorF_TX_PodState>());
}

void init() {
	can::registerMessageReceiver<can::messages::SensorF_TX_StatePod>(
			stateReceiver);
}

void update() {
}

}
