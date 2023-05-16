/*
 * Heartbeat.cpp
 *
 *  Created on: 02.12.2020
 *      Author: Felix
 */

#include <Heartbeat.hpp>
#include "cmsis_os.h"
#include "log.h"
#include "dbc_parser.hpp"
#include "message_buffer.h"

Heartbeat* Heartbeat::hb = nullptr;

Heartbeat::Heartbeat() {
}

Heartbeat::~Heartbeat() {
}


void Heartbeat::sendHeartbeat(void *params) {
	TxMessage hbTxMessage;
	hbTxMessage.txHeader.StdId = can::messages::CANZERO_Heartbeat::id;
	hbTxMessage.txHeader.DLC = can::messages::CANZERO_Heartbeat::dlc;
	extern osMessageQId czSendQueue;
	while (1) {
		hbTxMessage.txBuf[0] = (uint8_t) cz_interface::getInstance()->getStatus();
		osMessageQueuePut(czSendQueue, &hbTxMessage, 0, 0);
		osDelay(pdMS_TO_TICKS(Heartbeat::getHeartbeatInstance()->OD_HeartbeatInterval_get()));
	}

}
void Heartbeat::consumeHeartbeat(void *params) {
	RxMessage message;
	extern MessageBufferHandle_t heartbeatMessageBuffer;

	while (1) {
		if (xMessageBufferReceive( heartbeatMessageBuffer, &message, sizeof(message), pdMS_TO_TICKS(Heartbeat::getHeartbeatInstance()->OD_HeartbeatInterval_get() + 5)) != 0) {
			cz_interface::getInstance()->setStatus((cz_status)message.rxBuf[0]);
		}
		else{
			//currently no HB receiving
			//printDebug("Heartbeat timeout!\n");
		}
	}
}

void Heartbeat::OD_HeartbeatInterval_set(const uint16_t value) {
	extern osMutexId_t hbIntervalMutex;
	osMutexAcquire(hbIntervalMutex, osWaitForever);
	interval = value;
	osMutexRelease(hbIntervalMutex);
}

uint16_t Heartbeat::OD_HeartbeatInterval_get() {
	extern osMutexId_t hbIntervalMutex;
	osMutexAcquire(hbIntervalMutex, portMAX_DELAY);
	uint16_t returnValue = interval;
	osMutexRelease(hbIntervalMutex);
	return returnValue;
}

Heartbeat* Heartbeat::getHeartbeatInstance(){

	if(hb==nullptr){
		hb = new Heartbeat();
	}
	return hb;
}
