/*
 * pdu_control.hpp
 *
 *  Created on: 11 May 2023
 *      Author: karl
 */

#include "pdu_control.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "canzero.hpp"

namespace pdu {

struct PduStatus {
	bool m_enabled;
	bool m_pehwEnabled;

	bool operator==(const PduStatus& other)const  {
		return m_enabled == other.m_enabled
				&& m_pehwEnabled == other.m_pehwEnabled;
	}
	bool operator!=(const PduStatus& other)const  {
		return not (*this == other);
	}
};

static PduStatus s_status;
static PduStatus s_confirmed_status;

static bool m_error;
static bool m_resetError;


struct LpChannelConfig {
	static constexpr size_t NUMBER_OF_CHANNELS = 10;
	float m_duty[NUMBER_OF_CHANNELS];


	bool operator==(const LpChannelConfig& other) const {
		for(size_t i=0;i<NUMBER_OF_CHANNELS;i++){
			if(m_duty[i] != other.m_duty[i])return false;
		}
		return true;
	}

	bool operator!=(const LpChannelConfig& other) const {
		return not (*this == other);
	}
};

static LpChannelConfig s_lpChannelConfig;
static LpChannelConfig s_confirmed_lpChannelConfig;

struct HpChannelConfig {
	static constexpr size_t NUMBER_OF_CHANNELS = 4;
	float m_duty[NUMBER_OF_CHANNELS];

	bool operator==(const HpChannelConfig& other)const{
		for(size_t i=0;i<NUMBER_OF_CHANNELS;i++){
			if(m_duty[i] != other.m_duty[i])return false;
		}
		return false;
	}
	bool operator!=(const HpChannelConfig& other)const{
		return not (*this == other);
	}
};

static HpChannelConfig s_hpChannelConfig;
static HpChannelConfig s_confirmed_hpChannelConfig;


void enableChannel(HpChannel channel){
	s_hpChannelConfig.m_duty[static_cast<uint16_t>(channel)] = 100;
}

void disableChannel(HpChannel channel){
	s_hpChannelConfig.m_duty[static_cast<uint16_t>(channel)] = 0;
}

void enableChannel(LpChannel channel){
	s_lpChannelConfig.m_duty[static_cast<uint16_t>(channel)] = 100;
}

void disableChannel(LpChannel channel){
	s_lpChannelConfig.m_duty[static_cast<uint16_t>(channel)] = 0;
}

ChannelStatus getChannelStatus(LpChannel channel){
	float target = s_lpChannelConfig.m_duty[static_cast<uint16_t>(channel)];
	float real = s_confirmed_lpChannelConfig.m_duty[static_cast<uint16_t>(channel)];
	if(target == real){
		if(target != 0) {
			return CHANNEL_STATUS_ON;
		}else{
			return CHANNEL_STATUS_OFF;
		}
	}else {
		if(target != 0){
			return CHANNEL_STATUS_PENDING_ON;
		}else {
			return CHANNEL_STATUS_PENDING_OFF;
		}
	}
}
ChannelStatus getChannelStatus(HpChannel channel){
	float target = s_hpChannelConfig.m_duty[static_cast<uint16_t>(channel)];
	float real = s_confirmed_hpChannelConfig.m_duty[static_cast<uint16_t>(channel)];
	if(target == real){
		if(target != 0) {
			return CHANNEL_STATUS_ON;
		}else{
			return CHANNEL_STATUS_OFF;
		}
	}else {
		if(target != 0){
			return CHANNEL_STATUS_PENDING_ON;
		}else {
			return CHANNEL_STATUS_PENDING_OFF;
		}
	}
}


void receiveTxStatus(RxMessage& raw) {
	can::Message<can::messages::PDU_TX_Status> msg;
	s_confirmed_status.m_enabled = msg.get<can::signals::PDU_TX_Enabled>();
	s_confirmed_status.m_pehwEnabled = msg.get<can::signals::PDU_TX_PEHWEnabled>();
}

void init(){
	can::registerMessageReceiver<can::messages::PDU_TX_Status>(receiveTxStatus);
}


void update(){
	if(s_status != s_confirmed_status){
		can::Message<can::messages::PDU_RX_Control> controlMsg;
		controlMsg.set<can::signals::PDU_RX_Enable>(s_confirmed_status.m_pehwEnabled);
		//controlMsg.set<can::signals::PDU_RX_ErrorReset>(s_confirmed_status.m_error);
		controlMsg.set<can::signals::PDU_RX_PEHWEnable>(s_confirmed_status.m_pehwEnabled);
		controlMsg.send();
	}
	if(s_lpChannelConfig != s_confirmed_lpChannelConfig){
		//update lp channel config.
		//can::message<can::messages::PDU_LP_Duty> lpDutyMsg;
		//TODO ...

	}

	if(s_hpChannelConfig != s_confirmed_hpChannelConfig){
		// update hp channel config.
		//can::message<can::messages::PDU_HP_D_Duty> lpDutyMsg;
	}
}

}
