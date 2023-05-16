/*
 * handles.h
 *
 * This header should ONLY be included by the main.cpp. Otherwise linkage errors will occur!
 *
 *  Created on: 19.01.2021
 *      Author: Felix
 */
#pragma once

#include "cmsis_os.h"
#include "message_buffer.h"
#include "typedefinitions.h"

//TaskHandle_t canTaskHandle;
osThreadId_t heartbeatProduceHandle;
osThreadId_t heartbeatConsumeTaskHandle;
osThreadId_t emergencyTaskHandle;
osThreadId_t canReceiveTaskHandle;
osThreadId_t canSendTaskHandle;
osThreadId_t runtimeStatsHandle;
osThreadId_t sendOdEntriesTaskHandle;


constexpr size_t MSG_BUFFER_NUM_MESSAGES = 5;	// Number of messages to store in the message buffer
constexpr size_t MSG_BUFFER_SIZE = MSG_BUFFER_NUM_MESSAGES * (sizeof(RxMessage) + 4); 	// 4 bytes overhead to store the size_t

//Messagebuffer handles
MessageBufferHandle_t heartbeatMessageBuffer = xMessageBufferCreate(MSG_BUFFER_SIZE);
MessageBufferHandle_t emergencyMessageBuffer = xMessageBufferCreate(MSG_BUFFER_SIZE);
MessageBufferHandle_t handlePduRxMessageBuffer = xMessageBufferCreate(MSG_BUFFER_SIZE);
MessageBufferHandle_t handlePodStateMessageBuffer = xMessageBufferCreate(MSG_BUFFER_SIZE);
MessageBufferHandle_t handleSensorRMessageBuffer = xMessageBufferCreate(MSG_BUFFER_SIZE);

//mutex handles
const osMutexAttr_t nodeStateMutexAttr = {"nodeStateMutex",(osMutexPrioInherit|osMutexRobust),NULL,0,};
osMutexId_t nodeStateMutex = osMutexNew(&nodeStateMutexAttr);
const osMutexAttr_t hbIntervalMutexAttr = {"hbIntervalMutex",(osMutexPrioInherit|osMutexRobust),NULL,0,};
osMutexId_t hbIntervalMutex = osMutexNew(&hbIntervalMutexAttr);

//Queue handles
const osMessageQueueAttr_t czSendQueueAttr = {"czSendQueue",0,NULL,0,NULL,0};
osMessageQueueId_t czSendQueue = osMessageQueueNew(16, sizeof(TxMessage), &czSendQueueAttr);
const osMessageQueueAttr_t czReceiveQueueAttr = {"czReceiveQueue",0,NULL,0,NULL,0};
osMessageQueueId_t czReceiveQueue = osMessageQueueNew(16, sizeof(RxMessage), &czReceiveQueueAttr);

