/*
 * mdb_remote.hpp
 *
 *  Created on: May 20, 2023
 *      Author: OfficeLaptop
 */

#include "mdb_remote.hpp"
#include "canzero.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <cinttypes>
#include "estdio.hpp"

namespace mdb {


static volatile float m_airGap[MDB_COUNT];
static volatile float m_temp[MDB_COUNT];
static volatile uint8_t m_stateCount[MDB_STATE_COUNT];
static volatile PublicState m_state[MDB_COUNT];

static float m_targetAirGap = 10.0f;

static volatile PublicState m_publicState;

static volatile TickType_t m_lastStateBrd = 0;
static constexpr TickType_t STATE_BRD_INTERVAL = pdMS_TO_TICKS(50);

static Command m_command = MDB_COMMAND_NONE;
static TickType_t m_lastCommandBrd = 0;
static constexpr TickType_t COMMAND_BRD_INTERVAL = pdMS_TO_TICKS(100);

static PublicState m_stateMapping[MDB_STATE_COUNT];

template<uint8_t MDB_ID>
void mdbStatusReceiver(RxMessage& raw){
	// intertestingly this doesn't require synchronization because it will always be accessed by the same thread.
	uint8_t state = raw.rxBuf[0];
	PublicState publicState = m_stateMapping[state];
	uint8_t publicStateIdx = static_cast<uint8_t>(publicState);
	uint8_t prevStateIdx = static_cast<uint8_t>(m_state[MDB_ID]);

	m_stateCount[prevStateIdx]--;
	m_stateCount[publicStateIdx]++;
	m_state[MDB_ID] = publicState;

	if(m_stateCount[state] == MDB_COUNT){
		m_publicState = static_cast<PublicState>(state);
	}else{
		if(m_stateCount[MDB_ERROR]){
			m_publicState = MDB_ERROR;
		}else{
			m_publicState = MDB_INCONSISTANT;
		}
	}
	m_lastStateBrd = xTaskGetTickCount();
}

template<uint8_t MDB_ID>
void mdbAirGapReceiver(RxMessage& raw){
	m_airGap[MDB_ID] = *reinterpret_cast<float*>(raw.rxBuf);
}

template<uint8_t MDB_ID>
void mdbTempReceiver(RxMessage& raw){
	m_temp[MDB_ID] = *reinterpret_cast<float*>(raw.rxBuf);
}

void mdbActionRequestReceiver(RxMessage& raw){
	can::Message<can::messages::CLU_RX_ActionRequest> msg{raw};
	uint8_t actionRequestByte = msg.get<can::signals::CLU_RX_ActionRequest>();
	setCommand(static_cast<Command>(actionRequestByte));
}

void init(){
	m_stateMapping[MDB_STATE_INIT] = PublicState::MDB_IDLE;
	m_stateMapping[MDB_STATE_IDLE] = PublicState::MDB_IDLE;
	m_stateMapping[MDB_STATE_PRECHARGE] = PublicState::MDB_PRECHARGE;
	m_stateMapping[MDB_STATE_READY] = PublicState::MDB_READY;
	m_stateMapping[MDB_STATE_LEVI_START] = PublicState::MDB_LEVI_START;
	m_stateMapping[MDB_STATE_LEVI_RUN] = PublicState::MDB_LEVI;
	m_stateMapping[MDB_STATE_LEVI_END] = PublicState::MDB_LEVI;
	m_stateMapping[MDB_STATE_LEVI_UNSTABLE] = PublicState::MDB_LEVI;
	m_stateMapping[MDB_STATE_ERROR] = PublicState::MDB_ERROR;
	m_stateMapping[MDB_ERROR_OVERCURRENT] = PublicState::MDB_ERROR;
	m_stateMapping[MDB_ERROR_OVERVOLT] = PublicState::MDB_ERROR;
	m_stateMapping[MDB_ERROR_OVERTEMP] = PublicState::MDB_ERROR;
	m_stateMapping[MDB_ERROR_AIRGAP] = PublicState::MDB_ERROR;
	m_stateMapping[MDB_ERROR_AIRGAP_SEN] = PublicState::MDB_ERROR;
	m_stateMapping[MDB_ERROR_CAN] = PublicState::MDB_ERROR;
	m_stateMapping[MDB_ERROR_CURRE_CALIB] = PublicState::MDB_ERROR;
	m_stateMapping[MDB_ERROR_CHIPTEMP] = PublicState::MDB_ERROR;
	m_stateMapping[MDB_ERROR_CALCTIME] = PublicState::MDB_ERROR;

	for(uint8_t i=0;i<MDB_COUNT;i++){
		m_state[i] = MDB_OFF;
	}
	for(uint8_t i=0;i<MDB_STATE_COUNT;i++){
		m_stateCount[i] = 0;
	}
	m_stateCount[static_cast<uint8_t>(MDB_OFF)] = MDB_COUNT;
	m_publicState = MDB_OFF;

	can::registerMessageReceiver<can::messages::MDB1_TX_State>(mdbStatusReceiver<0>);
	can::registerMessageReceiver<can::messages::MDB2_TX_State>(mdbStatusReceiver<1>);
	can::registerMessageReceiver<can::messages::MDB3_TX_State>(mdbStatusReceiver<2>);
	can::registerMessageReceiver<can::messages::MDB4_TX_State>(mdbStatusReceiver<3>);
	can::registerMessageReceiver<can::messages::MDB5_TX_State>(mdbStatusReceiver<4>);
	can::registerMessageReceiver<can::messages::MDB6_TX_State>(mdbStatusReceiver<5>);

	can::registerMessageReceiver<can::messages::MDB1_TX_AirGap>(mdbAirGapReceiver<0>);
	can::registerMessageReceiver<can::messages::MDB2_TX_AirGap>(mdbAirGapReceiver<1>);
	can::registerMessageReceiver<can::messages::MDB3_TX_AirGap>(mdbAirGapReceiver<2>);
	can::registerMessageReceiver<can::messages::MDB4_TX_AirGap>(mdbAirGapReceiver<3>);
	can::registerMessageReceiver<can::messages::MDB5_TX_AirGap>(mdbAirGapReceiver<4>);
	can::registerMessageReceiver<can::messages::MDB6_TX_AirGap>(mdbAirGapReceiver<5>);

	can::registerMessageReceiver<can::messages::MDB1_TX_Temperature>(mdbTempReceiver<0>);
	can::registerMessageReceiver<can::messages::MDB2_TX_Temperature>(mdbTempReceiver<1>);
	can::registerMessageReceiver<can::messages::MDB3_TX_Temperature>(mdbTempReceiver<2>);
	can::registerMessageReceiver<can::messages::MDB4_TX_Temperature>(mdbTempReceiver<3>);
	can::registerMessageReceiver<can::messages::MDB5_TX_Temperature>(mdbTempReceiver<4>);
	can::registerMessageReceiver<can::messages::MDB6_TX_Temperature>(mdbTempReceiver<5>);
}

PublicState getState(){
	return m_publicState;
}

void setTargetAirGap(float airGap){
	if(airGap != m_targetAirGap){
		m_targetAirGap = airGap;
	}
}

float getAirGap(uint8_t mdbId){
	return m_airGap[mdbId];
}

float getTemperature(uint8_t mdbId){
	return m_temp[mdbId];
}

void setCommand(Command command){
	m_command = command;
	m_lastCommandBrd = 0;
}

void update(){
	TickType_t timeSinceLastBrd = xTaskGetTickCount() - m_lastStateBrd;
	if(timeSinceLastBrd > STATE_BRD_INTERVAL){
		can::Message<can::messages::CLU_TX_LevitationState> msg;
		msg.set<can::signals::CLU_LevitationState>(m_publicState);
		msg.send();
		m_lastStateBrd = xTaskGetTickCount();
	}
	TickType_t timeSinceLastCommandBrd = xTaskGetTickCount() - m_lastCommandBrd;
	if(timeSinceLastCommandBrd > COMMAND_BRD_INTERVAL && m_command != MDB_COMMAND_NONE){
		can::Message<can::messages::CLU_TX_ActionRequest> msg;
		msg.set<can::signals::CLU_TX_ActionRequest>(static_cast<uint8_t>(m_command));
		msg.set<can::signals::CLU_TX_TargetAirGap>(*reinterpret_cast<uint32_t*>(&m_targetAirGap));
		//msg.send(can::buses::BUS1);
		m_lastCommandBrd = xTaskGetTickCount();
	}
}

}
