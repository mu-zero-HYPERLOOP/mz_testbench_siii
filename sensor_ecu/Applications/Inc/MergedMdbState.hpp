/*
 * MdbMergedState.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include "canzero.hpp"
#include <cinttypes>

enum MdbState : uint8_t{
	MDB_STATE_INCONSISTANT = 0,
	MDB_STATE_PRECHARGE = 1,
	MDB_STATE_LEVITATION = 2,
	MDB_STATE_GROUNDED = 3,
	MDB_STATE_SAVE_TO_APPROCH = 4,
	MDB_STATE_PRECHARGE_DONE = 5,
};

class MergedMdbState {
public:
	static const MergedMdbState& getInstance(){
		static MergedMdbState instance;
		return instance;
	}

	MergedMdbState(MergedMdbState&) = delete;
	MergedMdbState(MergedMdbState&&) = delete;
	MergedMdbState& operator=(MergedMdbState&&) = delete;
	MergedMdbState& operator=(MergedMdbState&) = delete;

	MdbState getState() const{
		return m_state;
	}

private:
	MergedMdbState() {
		m_receiverId = can::registerMessageReceiver<can::messages::MDB_TX_State>([this](RxMessage& msg){
			this->callback(msg);
		});
	}
	~MergedMdbState(){
		can::unregisterMessageReceiver(m_receiverId);
	}


private:

	void updateState(){
		taskENTER_CRITICAL();
		for(unsigned int i=1;i<6;i++){
			if(m_states[0] != m_states[i]){
				m_state = MDB_STATE_INCONSISTANT;
				taskEXIT_CRITICAL();
				return;
			}
		}
		m_state = m_states[0];
		taskEXIT_CRITICAL();
	}

	void callback(RxMessage& raw){
		can::Message<can::messages::MDB_TX_State> msg {raw};
		uint8_t mdbId = msg.get<can::signals::MDB_Id>();
		uint8_t mdbStateId = msg.get<can::signals::MDB_State>();
		MdbState mdbState = static_cast<MdbState>(mdbStateId);
		m_states[mdbId] = mdbState;
		updateState();
	}
	unsigned int m_receiverId;
	MdbState m_states[6];

	MdbState m_state = MDB_STATE_INCONSISTANT;
};
