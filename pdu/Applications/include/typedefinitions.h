/*
 * typedefinitions.h
 *
 *  Created on: 08.12.2020
 *      Author: Felix
 */

#ifndef INCLUDE_TYPEDEFINITIONS_H_
#define INCLUDE_TYPEDEFINITIONS_H_

#include "can.h"

struct RxMessage {
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxBuf[8];
};

struct TxMessage {
	CAN_TxHeaderTypeDef txHeader = {0x00,0x00,CAN_ID_STD,CAN_RTR_DATA,0,DISABLE};
	uint8_t txBuf[8];
};

typedef enum {
	stopped = 4,
	operational = 5,
	pre_operational = 127,
	reset = 128
}cz_status;


#endif /* INCLUDE_TYPEDEFINITIONS_H_ */
