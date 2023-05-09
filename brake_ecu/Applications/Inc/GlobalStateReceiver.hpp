/*
 * GlobalStateReceiver.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include "canzero.hpp"


enum State : uint8_t{
            POD_OFF = 0,
            POD_IDLE = 1,
            POD_LAUNCH_PREPARATION = 2,
            POD_FAULT = 3,
            POD_READY_TO_LAUNCH = 4,
            POD_LAUNCHING = 5,
            POD_PUSHABLE = 6,
            POD_SAFE_TO_APPROACH = 7
};

class GlobalStateReceiver {
public:
	static const GlobalStateReceiver& getInstance() {
		static GlobalStateReceiver instance;
		return instance;
	}

	GlobalStateReceiver(GlobalStateReceiver&) = delete;
	GlobalStateReceiver(GlobalStateReceiver&&) = delete;
	GlobalStateReceiver operator=(GlobalStateReceiver&) = delete;
	GlobalStateReceiver operator=(GlobalStateReceiver&&) = delete;

	State getState() const{
		return m_state;
	}

private:
	explicit GlobalStateReceiver() {
		m_receiverId = can::registerMessageReceiver<can::messages::SensorF_TX_StatePod>([this](RxMessage& msg){
			this->callback(msg);
		});
	}

	~GlobalStateReceiver(){
		can::unregisterMessageReceiver(m_receiverId);
	}

	void callback(RxMessage& raw) {
		can::Message<can::messages::SensorF_TX_StatePod> msg {raw};
		uint8_t stateId = msg.get<can::signals::SensorF_TX_PodState>();
		m_state = static_cast<State>(stateId);
	}

	unsigned int m_receiverId;
	State m_state;

};
