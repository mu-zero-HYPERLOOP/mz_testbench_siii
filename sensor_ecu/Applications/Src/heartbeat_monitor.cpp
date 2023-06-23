/*
 * heatbeat_monitor.hpp
 *
 *  Created on: Jun 18, 2023
 *      Author: OfficeLaptop
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "heartbeat_monitor.hpp"
#include "canzero.hpp"
#include "state_maschine.hpp"

namespace heartbeat {

constexpr TickType_t HEARTBEAT_TIMEOUT = pdMS_TO_TICKS(500);
TickType_t lastBecuHeartbeat;
TickType_t lastCluHeartbeat;
TickType_t lastPduHeartbeat;
TickType_t lastTelHeartbeat;

void becuHeartbeatReceiver(RxMessage& raw){
	lastBecuHeartbeat = xTaskGetTickCount();
}

void cluHeartbeatReceiver(RxMessage& raw){
	lastCluHeartbeat = xTaskGetTickCount();
}

void pduHeartbeatReceiver(RxMessage& raw){
	lastPduHeartbeat = xTaskGetTickCount();
}

void telHeartbeatReceiver(RxMessage& raw){
	lastTelHeartbeat = xTaskGetTickCount();
}

void init(){
	can::registerMessageReceiver<can::messages::BrakeF_Heartbeat>(becuHeartbeatReceiver);
	can::registerMessageReceiver<can::messages::CLU_Heartbeat>(cluHeartbeatReceiver);
	can::registerMessageReceiver<can::messages::PDU_Heartbeat>(pduHeartbeatReceiver);
	can::registerMessageReceiver<can::messages::TelemetryNode_Heartbeat>(telHeartbeatReceiver);
}

bool telemetryConnected(){
	TickType_t timeSinceTelHeartbeat = xTaskGetTickCount() - lastTelHeartbeat;
	if(timeSinceTelHeartbeat > HEARTBEAT_TIMEOUT){
		return false;
	}else {
		return true;
	}
}

void update(){
	TickType_t timeSinceBecuHeartbeat = xTaskGetTickCount() - lastBecuHeartbeat;
	if(timeSinceBecuHeartbeat > HEARTBEAT_TIMEOUT){
		ERR_BECUHeartbeatMiss_set();
	}else{
		ERR_BECUHeartbeatMiss_clear();
	}
	TickType_t timeSinceCluHeartbeat = xTaskGetTickCount() - lastCluHeartbeat;
	if(timeSinceCluHeartbeat > HEARTBEAT_TIMEOUT){
		ERR_CLUHeartbeatMiss_set();
	}else{
		ERR_CLUHeartbeatMiss_clear();
	}
	TickType_t timeSincePduHeartbeat = xTaskGetTickCount() - lastPduHeartbeat;
	if(timeSincePduHeartbeat > HEARTBEAT_TIMEOUT){
		ERR_PDUHeartbeatMiss_set();
	}else{
		ERR_PDUHeartbeatMiss_clear();
	}
	TickType_t timeSinceTelHeartbeat = xTaskGetTickCount() - lastTelHeartbeat;
	if(timeSinceTelHeartbeat > HEARTBEAT_TIMEOUT && state_maschine::getState() != state_maschine::STATE::POD_STARTUP){
		ERR_TelemetryHeartbeatMiss_set();
	}else{
		ERR_TelemetryHeartbeatMiss_clear();
	}
}

}
