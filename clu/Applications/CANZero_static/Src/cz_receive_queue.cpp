/*
 * cz_receive_queue.cpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */
#include "cz_receive_queue.hpp"
#include "cz_typedefinitions.hpp"

static const osMessageQueueAttr_t czReceiveQueueAttr = {"czReceiveQueue",0,NULL,0,NULL,0};
osMessageQueueId_t czReceiveQueue = osMessageQueueNew(16, sizeof(RxMessage), &czReceiveQueueAttr);
