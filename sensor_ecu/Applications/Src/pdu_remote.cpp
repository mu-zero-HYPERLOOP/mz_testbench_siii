/*
 * pdu_control.hpp
 *
 *  Created on: 11 May 2023
 *      Author: karl
 */

#include <pdu_remote.hpp>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "canzero.hpp"
#include "estdio.hpp"

namespace pdu {

static bool g_minimizeMessages = false;

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

static PduStatus s_status = {
		.m_enabled = true,
		.m_pehwEnabled = false,
};
static PduStatus s_confirmed_status = {
		.m_enabled = false,
		.m_pehwEnabled = false,
};

struct LpChannelConfig {
	static constexpr size_t NUMBER_OF_CHANNELS = 10;
	bool m_status[NUMBER_OF_CHANNELS];


	bool operator==(const LpChannelConfig& other) const {
		for(size_t i=0;i<NUMBER_OF_CHANNELS;i++){
			if(m_status[i] != other.m_status[i])return false;
		}
		return true;
	}

	bool operator!=(const LpChannelConfig& other) const {
		return not (*this == other);
	}
};

static LpChannelConfig s_lpChannelConfig = {
		.m_status = {
				false,  //kistler
				false,
				true,  //ebox microcontrollers
				false,
				false, // mdbs, pi (broken) [dep]
				false, // mdbs, pi (backup) [dep]
				true, // telemetry.
				false,
				false,
				true //sdc.
		}
};
static LpChannelConfig s_confirmed_lpChannelConfig;

struct HpChannelConfig {
	static constexpr size_t NUMBER_OF_CHANNELS = 4;
	bool m_status[NUMBER_OF_CHANNELS];

	bool operator==(const HpChannelConfig& other)const{
		for(size_t i=0;i<NUMBER_OF_CHANNELS;i++){
			if(m_status[i] != other.m_status[i])return false;
		}
		return true;
	}
	bool operator!=(const HpChannelConfig& other)const{
		return not (*this == other);
	}
};

static HpChannelConfig s_hpChannelConfig = {
		.m_status = {false, true, false, false}
};
static HpChannelConfig s_confirmed_hpChannelConfig;

void enableChannel(HpChannel channel){
	s_hpChannelConfig.m_status[static_cast<uint16_t>(channel)] = true;
}

void disableChannel(HpChannel channel){
	s_hpChannelConfig.m_status[static_cast<uint16_t>(channel)] = false;
}

void enableChannel(LpChannel channel){
	if(channel == LP_CHANNEL8){
		printf("WARNING: Channel 8 might not work please use another channel!");
	}
	if(channel == LP_CHANNEL9){
		printf("WARNING: Channel 9 might not work please use another channel!");
	}
	s_lpChannelConfig.m_status[static_cast<uint16_t>(channel)] = true;;
}

void disableChannel(LpChannel channel){
	if(channel == LP_CHANNEL8){
		printf("WARNING: Channel 8 might not work please use another channel!");
	}
	if(channel == LP_CHANNEL9){
		printf("WARNING: Channel 9 might not work please use another channel!");
	}
	s_lpChannelConfig.m_status[static_cast<uint16_t>(channel)] = false;
}

ChannelStatus getChannelStatus(LpChannel channel){
	bool target = s_lpChannelConfig.m_status[static_cast<uint16_t>(channel)];
	bool real = s_confirmed_lpChannelConfig.m_status[static_cast<uint16_t>(channel)];
	if(target == real){
		if(target) {
			return CHANNEL_STATUS_ON;
		}else{
			return CHANNEL_STATUS_OFF;
		}
	}else {
		if(target){
			return CHANNEL_STATUS_PENDING_ON;
		}else {
			return CHANNEL_STATUS_PENDING_OFF;
		}
	}
}
ChannelStatus getChannelStatus(HpChannel channel){
	bool target = s_hpChannelConfig.m_status[static_cast<uint16_t>(channel)];
	bool real = s_confirmed_hpChannelConfig.m_status[static_cast<uint16_t>(channel)];
	if(target == real){
		if(target) {
			return CHANNEL_STATUS_ON;
		}else{
			return CHANNEL_STATUS_OFF;
		}
	}else {
		if(target){
			return CHANNEL_STATUS_PENDING_ON;
		}else {
			return CHANNEL_STATUS_PENDING_OFF;
		}
	}
}


void receiveTxStatus(RxMessage& raw) {
	can::Message<can::messages::PDU_TX_Status> msg {raw};
	s_confirmed_status.m_enabled = msg.get<can::signals::PDU_TX_Enabled>() == 1;
	s_confirmed_status.m_pehwEnabled = msg.get<can::signals::PDU_TX_PEHWEnabled>();
}

void receiveHpDutycycle(RxMessage& raw){
	can::Message<can::messages::PDU_TX_HP_Current> msg {raw};
	s_confirmed_hpChannelConfig.m_status[0] = msg.get<can::signals::PDU_HPCh1_Current>() > 0.1;
	s_confirmed_hpChannelConfig.m_status[1] = msg.get<can::signals::PDU_HPCh2_Current>() > 0.1;
	s_confirmed_hpChannelConfig.m_status[2] = msg.get<can::signals::PDU_HPCh3_Current>() > 0.1;
	s_confirmed_hpChannelConfig.m_status[3] = msg.get<can::signals::PDU_HPCh4_Current>() > 0.1;
}

void receiveLp1to5Dutycycle(RxMessage& raw){
	can::Message<can::messages::PDU_TX_LP_Current1> msg {raw};
	s_confirmed_lpChannelConfig.m_status[0] = msg.get<can::signals::PDU_LPCh1_Current>() > 0.1;
	s_confirmed_lpChannelConfig.m_status[1] = msg.get<can::signals::PDU_LPCh2_Current>() > 0.1;
	s_confirmed_lpChannelConfig.m_status[2] = msg.get<can::signals::PDU_LPCh3_Current>() > 0.1;
	s_confirmed_lpChannelConfig.m_status[3] = msg.get<can::signals::PDU_LPCh4_Current>() > 0.1;
	s_confirmed_lpChannelConfig.m_status[4] = msg.get<can::signals::PDU_LPCh5_Current>() > 0.1;
}

void receiveLp6to10Dutycycle(RxMessage& raw){
	can::Message<can::messages::PDU_TX_LP_Current2> msg {raw};
	s_confirmed_lpChannelConfig.m_status[5] = msg.get<can::signals::PDU_LPCh6_Current>() > 0.1;
	s_confirmed_lpChannelConfig.m_status[6] = msg.get<can::signals::PDU_LPCh7_Current>() > 0.1;
	s_confirmed_lpChannelConfig.m_status[7] = msg.get<can::signals::PDU_LPCh8_Current>() > 0.1;
	s_confirmed_lpChannelConfig.m_status[8] = msg.get<can::signals::PDU_LPCh9_Current>() > 0.1;
	s_confirmed_lpChannelConfig.m_status[9] = msg.get<can::signals::PDU_LPCh10_Current>() > 0.1;

}

void init(bool minimizeMessages){
	g_minimizeMessages = minimizeMessages;
	can::registerMessageReceiver<can::messages::PDU_TX_Status>(receiveTxStatus);
	can::registerMessageReceiver<can::messages::PDU_TX_HP_Current>(receiveHpDutycycle);
	can::registerMessageReceiver<can::messages::PDU_TX_LP_Current1>(receiveLp1to5Dutycycle);
	can::registerMessageReceiver<can::messages::PDU_TX_LP_Current2>(receiveLp6to10Dutycycle);

}

void killMe(){
	//disable the power channel of the sensor ecu itself. be very careful with this
	// might lead to problems where the ecus can't be flashed anymore because they turn off immediatly.
	disableChannel(LP_CHANNEL3);
}


void update(){
	if(not g_minimizeMessages || s_status != s_confirmed_status){
		can::Message<can::messages::PDU_RX_Control> controlMsg;
		controlMsg.set<can::signals::PDU_RX_Enable>(s_status.m_enabled);
		//controlMsg.set<can::signals::PDU_RX_ErrorReset>(s_confirmed_status.m_error);
		controlMsg.set<can::signals::PDU_RX_PEHWEnable>(s_status.m_pehwEnabled);
		controlMsg.send();
	}
	if(not g_minimizeMessages || s_lpChannelConfig != s_confirmed_lpChannelConfig){
		can::Message<can::messages::PDU_RX_LP_Dutycycle> lpMsg;
		lpMsg.set<can::signals::PDU_LPCh1_Dutycycle>(s_lpChannelConfig.m_status[0] ? 100.0 : 0.0);
		lpMsg.set<can::signals::PDU_LPCh2_Dutycycle>(s_lpChannelConfig.m_status[1] ? 100.0 : 0.0);
		lpMsg.set<can::signals::PDU_LPCh3_Dutycycle>(s_lpChannelConfig.m_status[2] ? 100.0 : 0.0);
		lpMsg.set<can::signals::PDU_LPCh8_Dutycycle>(s_lpChannelConfig.m_status[7] ? 100.0 : 0.0);
		lpMsg.set<can::signals::PDU_LPCh9_Dutycycle>(s_lpChannelConfig.m_status[8] ? 100.0 : 0.0);
		lpMsg.set<can::signals::PDU_LPCh10_Dutycycle>(s_lpChannelConfig.m_status[9] ? 100.0 : 0.0);
		lpMsg.send();
		can::Message<can::messages::PDU_RX_LP_Enable> lpEnableMsg;
		lpEnableMsg.set<can::signals::PDU_RX_LPCh4_Enable>(s_lpChannelConfig.m_status[3]);
		lpEnableMsg.set<can::signals::PDU_RX_LPCh5_Enable>(s_lpChannelConfig.m_status[4]);
		lpEnableMsg.set<can::signals::PDU_RX_LPCh6_Enable>(s_lpChannelConfig.m_status[5]);
		lpEnableMsg.set<can::signals::PDU_RX_LPCh7_Enable>(s_lpChannelConfig.m_status[6]);
		lpEnableMsg.send();
	}



	if(not g_minimizeMessages || s_hpChannelConfig != s_confirmed_hpChannelConfig){
		can::Message<can::messages::PDU_RX_HP_D_Dutycycle> hpMsg;
		hpMsg.set<can::signals::PDU_HPCh1_Dutycycle>(s_hpChannelConfig.m_status[0] ? 100.0 : 0.0);
		hpMsg.set<can::signals::PDU_HPCh2_Dutycycle>(s_hpChannelConfig.m_status[1] ? 100.0 : 0.0);
		// update hp channel config.
		hpMsg.send();
	}
}

}
