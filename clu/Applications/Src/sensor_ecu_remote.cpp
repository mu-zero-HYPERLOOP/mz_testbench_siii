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

static volatile PodState m_state;
static mdb::State m_mdbState;
constexpr TickType_t MESSAGE_LEVITATION_INTERVAL = 500;
static TickType_t m_lastMsgTime = 0;

static ActionRequest m_lastActionRequest = MDB_STOP;
static bool m_lastActionRequestDirty = true;
static TickType_t m_lastActionRequestTime = 0;
static constexpr TickType_t ACTION_REQUEST_INTERVALL = pdMS_TO_TICKS(50);


[[nodiscard]] ActionRequest getLastActionRequest() {
	return m_lastActionRequest;
}

PodState getState() {
	return m_state;
}

void stateReceiver(RxMessage &raw) {
	can::Message<can::messages::SensorF_TX_StatePod> msg { raw };
	m_state =
			static_cast<PodState>(msg.get<can::signals::SensorF_TX_PodState>());
}

void actionRequestReceiver(RxMessage &raw) {
	can::Message<can::messages::CLU_RX_ActionRequest> msg { raw };
	uint8_t action = msg.get<can::signals::CLU_RX_ActionRequest>();
	m_lastActionRequest = static_cast<ActionRequest>(action);
	m_lastActionRequestDirty = true;
}

void init() {
	can::registerMessageReceiver<can::messages::SensorF_TX_StatePod>(
			stateReceiver);
	can::registerMessageReceiver<can::messages::CLU_RX_ActionRequest>(
			actionRequestReceiver);
	m_mdbState = mdb::State::MDB_OFF;
	m_lastActionRequest = ActionRequest::MDB_STOP;
	m_lastMsgTime = 0;
}

void update() {
	// Send merged state of mdbs to sensor ecu.
	if (mdb::getState() != m_mdbState) {
		m_mdbState = mdb::getState();
		m_lastMsgTime = 0;
	}
	TickType_t ticksSinceLastMsg = xTaskGetTickCount() - m_lastMsgTime;
	if (ticksSinceLastMsg > MESSAGE_LEVITATION_INTERVAL) {
		ticksSinceLastMsg = xTaskGetTickCount();
		can::Message<can::messages::CLU_TX_LevitationState> msg;
		msg.set<can::signals::CLU_LevitationState>(m_mdbState);
		msg.send();
	}
	TickType_t ticksSinceActionRequestMsg = xTaskGetTickCount() - m_lastActionRequestTime;
	if(m_lastActionRequestDirty || ticksSinceActionRequestMsg > ACTION_REQUEST_INTERVALL){
		can::Message<can::messages::CLU_TX_ActionRequest> msg;
		msg.set<can::signals::CLU_TX_ActionRequest>(static_cast<uint8_t>(m_lastActionRequest));
		m_lastActionRequestDirty = false;
		m_lastActionRequestTime = xTaskGetTickCount();
	}
}

}
