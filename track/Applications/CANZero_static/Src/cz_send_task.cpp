/*
 * cz_send_task.cpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */
#include "cz_send_task.hpp"
#include "cz_handles.hpp"
#include "cz_typedefinitions.hpp"
#include "canzero_od.hpp"
#include "cz_log.hpp"
#include "cz_weak.hpp"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "cmsis_os.h"


void cz_send_entry(void* argv){
	TxMessage sendMessage;
	uint32_t TxMailbox;
	CAN_HandleTypeDef* hcanModule;

	unsigned int frame = 0;
	while(true){
		if(osMessageQueueGet(czSendQueue,&sendMessage,NULL,osWaitForever) != osOK){
			Error_Handler();
		}

		//uint32_t* buf = reinterpret_cast<uint32_t*>(&(sendMessage.txBuf[0]));
		//printf("%u\n", *buf);

		if(sendMessage.txHeader.DLC>8){
			Error_Handler();
		}

		// Extract the bus flag from the RTR field and then clear the flag bits
		can::CAN_BusTypeDef canBus = sendMessage.txHeader.RTR & can::buses::mask;
		sendMessage.txHeader.RTR &= ~(can::buses::mask);

		if(canBus == can::buses::BUS1) {
			hcanModule = &hcan1;
		} else if(canBus == can::buses::BUS2) {
			hcanModule = &hcan2;
		} else {
			// Decide on the ID
			if (sendMessage.txHeader.StdId % 2 == 0){
				hcanModule = &hcan1;
			} else{
				hcanModule = &hcan2;
			}
		}

		// If there is a mailbox free on the requested CAN, just send the message
		if(HAL_CAN_GetTxMailboxesFreeLevel(hcanModule) > 0) {
			if (HAL_CAN_AddTxMessage(hcanModule, &sendMessage.txHeader, sendMessage.txBuf, &TxMailbox) != HAL_OK) {
				/* Transmission request Error */
				Error_Handler();
			}
		} else {	// No mailbox free on requested CAN bus
			// Count delayed messages
			if(hcanModule == &hcan1) {
				OD_CAN1_DelayedTxMessages++;
				//printDebug("WARNING :: can1 tx mailbox overflow.\n")
			} else {
				OD_CAN2_DelayedTxMessages++;
				//printDebug("WARNING :: can2 tx mailbox overflow.\n")
			}

			osDelay(1);		// Use smallest delay possible

			// Try again
			if(HAL_CAN_GetTxMailboxesFreeLevel(hcanModule) > 0) {
				if (HAL_CAN_AddTxMessage(hcanModule, &sendMessage.txHeader, sendMessage.txBuf, &TxMailbox) != HAL_OK) {
					/* Transmission request Error */
					canzero::handle_trasmission_request_error();
				}
			} else {
				// Discard message
				if(hcanModule == &hcan1) {
					OD_CAN1_DiscardedTxMessages++;
					canzero::handle_txmailbox_overflow(&hcan1);
				} else {
					OD_CAN2_DiscardedTxMessages++;
					canzero::handle_txmailbox_overflow(&hcan2);
				}
			}
		}
		frame ++;
	}
}

