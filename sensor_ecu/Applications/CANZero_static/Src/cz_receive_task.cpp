/*
 * cz_receive_task.cpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */
#include "cz_receive_task.hpp"
#include "cz_handles.hpp"
#include "cz_typedefinitions.hpp"
#include "cz_processRx.hpp"
#include "canzero_od.hpp"
#include "dbc_parser.hpp"
#include "cz_log.hpp"

#include "FreeRTOS.h"
#include "message_buffer.h"
#include "estdio.hpp"


void cz_receive_task(void* argv) {
	RxMessage message;
	while (1) {
		osMessageQueueGet(czReceiveQueue, (void*) &message, NULL, osWaitForever);
		if(message.rxHeader.StdId == 1040){
			printf("HELLO !!! \n");
		}
		if (message.rxHeader.RTR == CAN_RTR_DATA) {
 			if(can::checkRxMessage<can::messages::CANZERO_BTL_RX>(message)) {
				if(message.rxBuf[0]==0xff) {
					NVIC_SystemReset();
				}
			} else if(can::checkRxMessage<can::messages::CANZERO_SDO_Req_Up>(message)) {
				handleSDORequestUpload(message);
			} else if(can::checkRxMessage<can::messages::CANZERO_SDO_Req_Down>(message)) {
				handleSDORequestDownload(message);
			} else {
				processRX(message);
			}
		}
		else if (message.rxHeader.RTR == CAN_RTR_REMOTE) {
			printDebug("RTR must not be used!\n");
		}
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	uint8_t RxData[8];
	CAN_RxHeaderTypeDef RxHeader;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	RxMessage m;
	m.rxHeader = RxHeader;
	for(int i = 0; i<8;i++)
		m.rxBuf[i] = RxData[i];
	//if the function hangs/does not return it might because the interrupts have invalid priorities.
	//They have to be greater or equal to the max interrupt priority (default: 5) set in the FreeRTOS config Parameters
	//Interrupt priority is set in HAL_CAN_MspInit in can.c
	if(osMessageQueuePut(czReceiveQueue, &m, 0, 0) != osOK){

	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	uint8_t RxData[8];
	CAN_RxHeaderTypeDef RxHeader;

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	RxMessage m;
	m.rxHeader = RxHeader;
	for(int i = 0; i<8;i++)
		m.rxBuf[i] = RxData[i];
	//if the function hangs/does not return it might because the interrupts have invalid priorities.
	//They have to be greater or equal to the max interrupt priority (default: 5) set in the FreeRTOS config Parameters
	//Interrupt priority is set in HAL_CAN_MspInit in can.c
	if(osMessageQueuePut(czReceiveQueue, &m, 0, 0) != osOK){

	}
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_ResetError(hcan);
	printDebugISR("CAN Errors got reseted!\n");
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	//LED_RGB_Write(100, 0, 0);
}
