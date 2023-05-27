/*
 * mdb_remote.hpp
 *
 *  Created on: May 20, 2023
 *      Author: OfficeLaptop
 */

#include "mdb_remote.hpp"
#include "canzero.hpp"
#include <cinttypes>

namespace mdb {


static volatile float m_airGap[MDB_COUNT];
static volatile float m_temp[MDB_COUNT];
static volatile uint8_t m_stateCount[MDB_STATE_COUNT];
static volatile State m_state[MDB_COUNT];

static float m_targetAirGap[MDB_COUNT];
static bool m_dirtyAirGap[MDB_COUNT];

static volatile State m_combinedState;

template<uint8_t MDB_ID>
void mdbStatusReceiver(RxMessage& raw){
	// intertestingly this doesn't require synchronization because it will always be accessed by the same thread.
	uint8_t state = raw.rxBuf[0];
	m_stateCount[m_state[MDB_ID]]--;
	m_stateCount[state]++;
	m_state[MDB_ID] = static_cast<State>(state);
	if(m_stateCount[state] == 6){
		m_combinedState = static_cast<State>(state);
	}else{
		m_combinedState = MDB_INCONSISTANT;
	}

}

template<uint8_t MDB_ID>
void mdbAirGapReceiver(RxMessage& raw){
	m_airGap[MDB_ID] = *reinterpret_cast<float*>(raw.rxBuf);
}

template<uint8_t MDB_ID>
void mdbTempReceiver(RxMessage& raw){
	m_temp[MDB_ID] = *reinterpret_cast<float*>(raw.rxBuf);
}

void init(){
	for(uint8_t i=0;i<MDB_COUNT;i++){
		m_state[i] = MDB_OFF;
	}
	for(uint8_t i=0;i<MDB_STATE_COUNT;i++){
		m_stateCount[i] = 0;
	}
	m_stateCount[static_cast<uint8_t>(MDB_OFF)] = MDB_COUNT;
	m_combinedState = MDB_OFF;

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

State getState(){
	return m_combinedState;
}

void setTargetAirGap(uint8_t mdbId, float airGap){
	if(m_targetAirGap[mdbId] != airGap){
		m_targetAirGap[mdbId] = airGap;
		m_dirtyAirGap[mdbId] = true;
	}
}

float getAirGap(uint8_t mdbId){
	return m_airGap[mdbId];
}

float getTemperature(uint8_t mdbId){
	return m_temp[mdbId];
}

void update(){
	if(m_dirtyAirGap[0]){
		can::Message<can::messages::MDB1_RX_TargetAirGap> msg;
		msg.set<can::signals::MDB1_TargetAirGap>(*reinterpret_cast<uint32_t*>(&(m_targetAirGap[0])));
		msg.send();
	}else if(m_dirtyAirGap[1]){
		can::Message<can::messages::MDB2_RX_TargetAirGap> msg;
		msg.set<can::signals::MDB2_TargetAirGap>(*reinterpret_cast<uint32_t*>(&(m_targetAirGap[1])));
		msg.send();
	}else if(m_dirtyAirGap[2]){
		can::Message<can::messages::MDB3_RX_TargetAirGap> msg;
		msg.set<can::signals::MDB3_TargetAirGap>(*reinterpret_cast<uint32_t*>(&(m_targetAirGap[2])));
		msg.send();
	}else if(m_dirtyAirGap[3]){
		can::Message<can::messages::MDB4_RX_TargetAirGap> msg;
		msg.set<can::signals::MDB4_TargetAirGap>(*reinterpret_cast<uint32_t*>(&(m_targetAirGap[3])));
		msg.send();
	}else if(m_dirtyAirGap[4]){
		can::Message<can::messages::MDB5_RX_TargetAirGap> msg;
		msg.set<can::signals::MDB5_TargetAirGap>(*reinterpret_cast<uint32_t*>(&(m_targetAirGap[4])));
		msg.send();
	}else if(m_dirtyAirGap[5]){
		can::Message<can::messages::MDB6_RX_TargetAirGap> msg;
		msg.set<can::signals::MDB6_TargetAirGap>(*reinterpret_cast<uint32_t*>(&(m_targetAirGap[5])));
		msg.send();
	}
}

}
