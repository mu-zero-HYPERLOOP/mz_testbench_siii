/**
 * @file cz_processRx.c
 *
 * @date 29.04.2022
 * @author: Max Moebius
 *
 * @brief contains processRX which handles node specific can messages
 */
 /**
  * @addtogroup CANzero
  * @{
  */
#include "cz_processRx.hpp"
#include <canzero_defines.h>
#include <cz_handles.hpp>
#include "dbc_parser.hpp"


static constexpr size_t MSG_BUFFER_NUM_MESSAGES = 5;	// Number of messages to store in the message buffer
static constexpr size_t MSG_BUFFER_SIZE = MSG_BUFFER_NUM_MESSAGES * (sizeof(RxMessage) + 4); 	// 4 bytes overhead to store the size_t
MessageBufferHandle_t handlePDO1MessageBuffer = xMessageBufferCreate(MSG_BUFFER_SIZE);

  /**
   * @brief function that process all node specific messages
   *
   * @param message
   */
void processRX(RxMessage message) {
	if (can::checkRxMessage<can::messages::CANZERO_RX_PDO1>(message)) {
		xMessageBufferSend(handlePDO1MessageBuffer, &message, sizeof(message), 10);
	}
	/*
	 else if (can::checkRxMessage<can::messages::SCIMO_PE_TX_Errors>(message)) {
		 xMessageBufferSend(SciMoPEMessageBuffer, &message, sizeof(message), 10);
	 }
	 else if (can::checkRxMessage<can::messages::SensorR_RX_RunControl>(message)) {
		 xMessageBufferSend(runControlMessageBuffer, &message, sizeof(message), 10);
	 }
	 else if (can::checkRxMessage<can::messages::OpticalSensor_TX_MainData>(message)) {
		 xMessageBufferSend(opticalSensorMessageBuffer, &message, sizeof(message), 10);
	 }
	 else if (can::checkRxMessage<can::messages::SensorF_TX_AccFront>(message)) {
		 xMessageBufferSend(accFrontMessageBuffer, &message, sizeof(message), 10);
	 }
	 else if (can::checkRxMessage<can::messages::SensorF_TX_EncoderFront>(message)) {
		 xMessageBufferSend(encoderFrontMessageBuffer, &message, sizeof(message), 10);
	}
	// TODO löschen
	else if (can::checkRxMessage<can::messages::SensorR_TX_AccRear>(message)) {
			xMessageBufferSend(accFrontMessageBuffer, &message, sizeof(message), 10);
		}
	else {
		if (message.rxHeader.IDE == CAN_ID_STD) {
			printDebug("unknown/unhandled CAN Std-ID: %lu\n", message.rxHeader.StdId);
		}
		else {
			printDebug("unknown/unhandled CAN Ext-ID: %lu\n", message.rxHeader.ExtId);
		}
	}
	*/
}
/**
 * @}
 */
