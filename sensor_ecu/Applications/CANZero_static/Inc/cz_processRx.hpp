/*
 * cz_processRx.h
 *
 *  Created on: 12.04.2021
 *      Author: Felix
 */

#ifndef CANZERO_INCLUDE_CZ_PROCESSRX_H_
#define CANZERO_INCLUDE_CZ_PROCESSRX_H_

#include <cz_typedefinitions.hpp>
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "cz_RxMessageQueue.hpp"



extern MessageBufferHandle_t handlePDO1MessageBuffer;

void processRX(RxMessage message);


#endif /* CANZERO_INCLUDE_CZ_PROCESSRX_H_ */
