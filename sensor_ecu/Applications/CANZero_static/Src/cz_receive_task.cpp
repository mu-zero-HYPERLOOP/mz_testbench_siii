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
#include "log_dep.hpp"

void cz_receive_task(void* argv) {
	RxMessage message;
	while (1) {
		osMessageQueueGet(czReceiveQueue, (void*) &message, NULL, osWaitForever);
		logln("Received Message");

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

