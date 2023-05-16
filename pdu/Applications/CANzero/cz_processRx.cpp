/**
 * @file cz_processRx.c
 *
 * @date 12.04.2021
 * @author: Felix
 *
 * @brief contains processRX which handles node specific can messages
 */
/**
 * @addtogroup CANzero
 * @{
 */
#include "cz_processRx.hpp"
#include <cz_interface.hpp>
#include <canzero_defines.h>
/**
 * @brief function that process all node specific messages
 *
 * @param message
 */
void processRX(RxMessage message) {
	// All the message buffers
	extern MessageBufferHandle_t handlePduRxMessageBuffer;
	extern MessageBufferHandle_t handlePodStateMessageBuffer;
	extern MessageBufferHandle_t handleSensorRMessageBuffer;


	if (
		can::checkRxMessage<can::messages::PDU_RX_Control>(message) ||
		can::checkRxMessage<can::messages::PDU_RX_Manual_Control>(message) ||
		can::checkRxMessage<can::messages::PDU_RX_HP_D_Dutycycle>(message) ||
		can::checkRxMessage<can::messages::PDU_RX_LP_Dutycycle>(message) ||
		can::checkRxMessage<can::messages::PDU_RX_LP_Enable>(message)
//		can::checkRxMessage<can::messages::SensorR_TX_Temperature>(message) ||
//		can::checkRxMessage<can::messages::SensorF_TX_Temperature>(message)
	) {
		xMessageBufferSend(handlePduRxMessageBuffer, &message, sizeof(message), 1);
	}else {
		if(message.rxHeader.IDE == CAN_ID_STD) {
			printDebug("unknown/unhandled CAN Std-ID: %lu\n", message.rxHeader.StdId);
		} else {
			printDebug("unknown/unhandled CAN Ext-ID: %lu\n", message.rxHeader.ExtId);
		}
	}
}
/**
 * @}
 */
