/*
 * cz_send_queue.cpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */

#ifndef CANZERO_STATIC_SRC_CZ_SEND_QUEUE_CPP_
#define CANZERO_STATIC_SRC_CZ_SEND_QUEUE_CPP_

#include "cz_send_queue.hpp"
#include "cz_typedefinitions.hpp"


static const osMessageQueueAttr_t czSendQueueAttr = {"czSendQueue",0,NULL,0,NULL,0};

osMessageQueueId_t czSendQueue = osMessageQueueNew(16, sizeof(TxMessage), &czSendQueueAttr);

#endif /* CANZERO_STATIC_SRC_CZ_SEND_QUEUE_CPP_ */
