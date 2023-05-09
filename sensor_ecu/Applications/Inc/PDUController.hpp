/*
 * PDUController.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "canzero.hpp"

class PDUController {
public:

	static PDUController& getInstance(){
		static PDUController instance;
		return instance;
	}

	PDUController(PDUController&) = delete;
	PDUController(PDUController&&) = delete;
	PDUController& operator=(PDUController&) = delete;
	PDUController& operator=(PDUController&&) = delete;

	bool isEnabled() {
		return m_enabled;
	}

	bool hasError() {
		return m_error;
	}

	bool isHVEnabled(){
		return m_hvEnabled;
	}

	void enable(){
		//TODO this is super sketchy. reseting all errors.
		controllHelper(true, m_error, m_hvEnabled);
	}

	void disable(){
		//TODO this is super sketchy. reseting all errors.
		controllHelper(false, m_error, m_hvEnabled);
	}

	void resetErrors(){
		controllHelper(m_enabled, true, m_hvEnabled);
	}

	void enableHV(){
		//TODO this is super sketchy. reseting all errors.
		controllHelper(m_enabled, m_error, true);
	}

	void disableHV(){
		//TODO this is super sketchy. reseting all errors.
		controllHelper(m_enabled, m_error, false);
	}

private:
	explicit PDUController() {
		m_receiverId = can::registerMessageReceiver<can::messages::PDU_TX_Status>([this](RxMessage& msg){
			this->callback(msg);
		});
	}
	~PDUController(){
		can::unregisterMessageReceiver(m_receiverId);
	}

	void callback(RxMessage& raw){
		can::Message<can::messages::PDU_TX_Status> msg {raw};
		m_enabled = msg.get<can::signals::PDU_TX_Enabled>();
		m_error = msg.get<can::signals::PDU_TX_ErrorFlag>();
		m_hvEnabled = msg.get<can::signals::PDU_TX_PEHWEnabled>();
	}

	void controllHelper(bool enable, bool resetErrors, bool hvEnable){
		can::Message<can::messages::PDU_RX_Control> msg;
		msg.set<can::signals::PDU_RX_Enable>(enable);
		msg.set<can::signals::PDU_RX_ErrorReset>(resetErrors);
		msg.set<can::signals::PDU_RX_PEHWEnable>(hvEnable);
		msg.send();
	}

	void lpDutyControllHelper(float ch1, float ch10, float ch2, float ch3, float ch8, float ch9){
		//TODO implement.
	}

	void hpDutyDControllHelper(float hpch1, float hpch2, float d1, float d2, float d3, float d4){
		//TODO implement.
	}

	unsigned int m_receiverId;
	bool m_enabled;
	bool m_error;
	bool m_hvEnabled;

	float ch1;
	float ch10;
	float ch2;
	float ch3;
	float ch8;
	float ch9;
};
