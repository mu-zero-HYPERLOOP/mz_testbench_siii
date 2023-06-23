/*
 * error_recv.hpp
 *
 *  Created on: Jun 18, 2023
 *      Author: OfficeLaptop
 */
#include <error.hpp>
#include "canzero.hpp"
#include "state_maschine.hpp"

namespace errors {

constexpr uint8_t CPU_OVER_TEMP_PRIO = 10;
constexpr uint8_t OVER_VOLT_PRIO = 10;
constexpr uint8_t UNDER_VOLT_PRIO = 8;

constexpr uint8_t INVALID_POSITION_PRIO_LEVI = 15;
constexpr uint8_t INVALID_POSITION_PRIO_NON_LEVI = 5;

constexpr uint8_t RESERVOIR_OVER_TEMP_PRIO = 8;

constexpr uint8_t CLU_HEARTBEATMISS_PRIO = 20;
constexpr uint8_t BECU_HEARTBEATMISS_PRIO = 20;
constexpr uint8_t PDU_HEARTBEATMISS_PRIO = 20;
constexpr uint8_t TELEMETRY_MISS_PRIO = 20;

constexpr uint8_t TITAN_OVER_TEMP_PRIO = 10;
constexpr uint8_t HYPERION_OVER_TEMP_PRIO = 10;
constexpr uint8_t TITAN_LOWHP_PRIO = 10;
constexpr uint8_t HYPERION_LOWHP_PRIO = 10;
constexpr uint8_t TITAN_LOWCAP_PRIO = 10;
constexpr uint8_t HYPERION_LOWCAP_PRIO = 10;

uint8_t errorLevel = 0;
uint8_t ignoreLevel = 30;
uint8_t emergencyLevel = 0xFF;

uint8_t becuErrorLevel = 0;
uint8_t cluErrorLevel = 0;

void BECUErrorRecv(RxMessage& raw){
	can::Message<can::messages::BrakeF_TX_Error> msg{raw};
	becuErrorLevel = msg.get<can::signals::BrakeF_TX_ErrorPrio>();
}

void CLUErrorRecv(RxMessage& raw){
	can::Message<can::messages::CLU_TX_Error> msg{raw};
	cluErrorLevel = msg.get<can::signals::CLU_TX_ErrorPrio>();
}

void ignoreLevelRecv(RxMessage& raw){
	can::Message<can::messages::SensorF_RX_IgnoreLevel> msg{raw};
	ignoreLevel = msg.get<can::signals::SensorF_IgnoreLevel>();
	emergencyLevel = msg.get<can::signals::SensorF_EmergencyLevel>();
}

void init(){
	can::registerMessageReceiver<can::messages::BrakeF_TX_Error>(BECUErrorRecv);
	can::registerMessageReceiver<can::messages::CLU_TX_Error>(CLUErrorRecv);
	can::registerMessageReceiver<can::messages::SensorF_RX_IgnoreLevel>(ignoreLevelRecv);

	can::Message<can::messages::SensorF_EMCY> msg;
	msg.send();
}

bool hasError(){
	return errorLevel >= ignoreLevel || hasEmergency();
}

bool hasEmergency(){
	return errorLevel >= emergencyLevel;
}


void update(){
	errorLevel = std::max(becuErrorLevel, cluErrorLevel);

	if(ERR_CPUOverTemp_get()){
		errorLevel = std::max(CPU_OVER_TEMP_PRIO, errorLevel);
	}
	if(ERR_OverVolt_get()){
		errorLevel = std::max(OVER_VOLT_PRIO, errorLevel);
	}
	if(ERR_UnderVolt_get()){
		errorLevel = std::max(UNDER_VOLT_PRIO, errorLevel);
	}
	if(ERR_InvalidPosition_get()){
		state_maschine::PodState state = state_maschine::getState();
		using namespace state_maschine;
		if(state == STATE::POD_START_LEVITATION
				|| state == STATE::POD_STABLE_LEVITATION
				|| state == STATE::POD_DISENGAGE_BRAKES
				|| state == STATE::POD_STOP_LEVITATION){
			errorLevel = std::max(INVALID_POSITION_PRIO_LEVI, errorLevel);
		}else{
			errorLevel = std::max(INVALID_POSITION_PRIO_NON_LEVI, errorLevel);
		}
	}
	if(ERR_ReservoirOverTemp_get()){
		errorLevel = std::max(RESERVOIR_OVER_TEMP_PRIO, errorLevel);
	}
	if(ERR_CLUHeartbeatMiss_get()){
		errorLevel = std::max(CLU_HEARTBEATMISS_PRIO, errorLevel);
	}
	if(ERR_BECUHeartbeatMiss_get()){
		errorLevel = std::max(BECU_HEARTBEATMISS_PRIO, errorLevel);
	}
	if(ERR_PDUHeartbeatMiss_get()){
		errorLevel = std::max(PDU_HEARTBEATMISS_PRIO, errorLevel);
	}
	if(ERR_TelemetryHeartbeatMiss_get()){
		if(state_maschine::getState() != state_maschine::STATE::POD_STARTUP){
			errorLevel = std::max(TELEMETRY_MISS_PRIO, errorLevel);
		}
	}
	if(ERR_HyperionOverTemp_get()){
		errorLevel = std::max(HYPERION_OVER_TEMP_PRIO, errorLevel);
	}
	if(ERR_TitanOverTemp_get()){
		errorLevel = std::max(TITAN_OVER_TEMP_PRIO, errorLevel);
	}
	if(ERR_HyperionLowHp_get()){
		errorLevel = std::max(HYPERION_LOWHP_PRIO, errorLevel);
	}
	if(ERR_TitanLowHp_get()){
		errorLevel = std::max(TITAN_LOWHP_PRIO, errorLevel);
	}
	if(ERR_HyperionLowCap_get()){
		errorLevel = std::max(HYPERION_LOWCAP_PRIO, errorLevel);
	}
	if(ERR_TitanLowCap_get()){
		errorLevel = std::max(TITAN_LOWCAP_PRIO, errorLevel);
	}
}

}




