/*
 * cz_heartbeat.cpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */
#include "cz_heartbeat.hpp"
#include "canzero_od.hpp"
#include "cz_statemachine.hpp"
#include "cz_send_queue.hpp"
#include "cz_log.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "message_buffer.h"


//TODO remove me
#include "log_dep.hpp"

static constexpr size_t MSG_BUFFER_NUM_MESSAGES = 5;	// Number of messages to store in the message buffer
static constexpr size_t MSG_BUFFER_SIZE = MSG_BUFFER_NUM_MESSAGES * (sizeof(RxMessage) + 4); 	// 4 bytes overhead to store the size_t

static MessageBufferHandle_t heartbeatMessageBuffer = xMessageBufferCreate(MSG_BUFFER_SIZE);

void canzero::heartbeat::producer_entry(void* argv){
	//TODO setup callback logic for processRx.

	TxMessage hbTxMessage;
	hbTxMessage.txHeader.StdId = can::messages::CANZERO_Heartbeat::id;
	hbTxMessage.txHeader.DLC = can::messages::CANZERO_Heartbeat::dlc;
	unsigned int frame = 0;
	while (1) {
		hbTxMessage.txBuf[0] = (uint8_t) canzero::getStatus();
		osMessageQueuePut(czSendQueue, &hbTxMessage, 0, 0);
		osDelay(pdMS_TO_TICKS(canzero::heartbeat::getInterval()));
		frame ++;
	}
}

void canzero::heartbeat::consumer_entry(void* argv){
	RxMessage message;
	while (true) {
		if (xMessageBufferReceive( heartbeatMessageBuffer, &message, sizeof(message),
				pdMS_TO_TICKS(canzero::heartbeat::getInterval() + 5)) != 0) {
			canzero::setStatus((cz_status)message.rxBuf[0]);
		}
		else{
			//currently no HB receiving
			//printDebug("Heartbeat timeout!\n");
		}
	}
}

void canzero::heartbeat::setInterval(uint16_t value){
	OD_HeartbeatInterval_set(value);
}

uint16_t canzero::heartbeat::getInterval(){
	return OD_HeartbeatInterval_get();
}


