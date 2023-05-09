/*
 * BrakeECUReceiver.hpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "canzero.hpp"

enum BrakeState : uint8_t {
	BRAKE_UNDEFINED = 0, BRAKE_ENGAGED = 1, BRAKE_DISENGAGED = 2
};

class BrakeECUController {
public:
	static BrakeECUController& getInstance() {
		static BrakeECUController instance;
		return instance;
	}

	~BrakeECUController() {
		can::unregisterMessageReceiver(m_receiverId);
	}
	BrakeECUController(BrakeECUController&) = delete;
	BrakeECUController(BrakeECUController&&) = delete;
	BrakeECUController& operator=(BrakeECUController&) = delete;
	BrakeECUController& operator=(BrakeECUController&&) = delete;

	BrakeState getBrakeState() {
		return m_brakeState;
	}

	void enable() {
		m_enableTarget = true;
		controlHelper();
	}

	void disable() {
		m_enableTarget = false;
		controlHelper();
	}

	void engageBrakes() {
		m_engageTarget = true;
		controlHelper();
	}

	void disengageBrakes() {
		m_engageTarget = false;
		controlHelper();
	}

	bool isEnabled() {
		return m_enable;
	}

	bool isEngaged() {
		return m_engage;
	}

	bool hasError() {
		return m_error;
	}

private:

	void controlHelper() {
		can::Message<can::messages::BrakeF_RX_Control> msg;
		msg.set<can::signals::BrakeF_RX_Enable>(m_enableTarget);
		uint8_t engageEnum =
				m_engageTarget ?
						can::signals::BrakeF_RX_Engage::DISENGAGE :
						can::signals::BrakeF_RX_Engage::ENGAGEEMERGENCY;
		msg.set<can::signals::BrakeF_RX_Engage>(engageEnum);
		msg.set<can::signals::BrakeF_RX_ErrorReset>(m_error); //FIXME BAD BAD BAD.
		msg.send();
	}

	void callback(RxMessage &raw) {
		can::Message<can::messages::BrakeF_TX_Status> msg { raw };
		m_enable = msg.get<can::signals::BrakeF_TX_Enabled>();
		m_engage = msg.get<can::signals::BrakeF_TX_Status>() > 0;
		if(m_engage){
			m_brakeState = BRAKE_ENGAGED;
		}else{
			m_brakeState = BRAKE_DISENGAGED;
		}
		m_error = msg.get<can::signals::BrakeF_TX_ErrorFlag>();
	}

	explicit BrakeECUController() {
		m_receiverId = can::registerMessageReceiver<can::messages::BrakeF_TX_Status>([this](RxMessage& raw){
			this->callback(raw);
		});
	}

	unsigned int m_receiverId;
	BrakeState m_brakeState = BRAKE_UNDEFINED;

	bool m_enable;
	bool m_engage;
	bool m_error;

	bool m_enableTarget;
	bool m_engageTarget;

};
