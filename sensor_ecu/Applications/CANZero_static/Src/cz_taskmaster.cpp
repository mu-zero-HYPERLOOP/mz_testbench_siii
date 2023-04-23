/*
 * cz_taskmaster.cpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */

#include "cz_send_task.hpp"
#include "cz_receive_task.hpp"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cz_statemachine.hpp"
#include "cz_heartbeat.hpp"
#include "cz_emergency.hpp"
#include "cz_log.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void canzero_start(void *argv) {
	canzero::init();

	TaskHandle_t sendTaskHandle = nullptr;
	BaseType_t sendStatus = xTaskCreate(cz_send_entry, "cz_send", 256, NULL,
			osPriorityNormal, &sendTaskHandle);
	if (sendStatus != pdPASS) {
		printDebug("Failed to start canzero send task");
	}

	TaskHandle_t recvTaskHandle = nullptr;
	BaseType_t recvStatus = xTaskCreate(cz_receive_task, "cz_receive", 256, NULL,
			osPriorityNormal, &recvTaskHandle);
	if (recvStatus != pdPASS) {
		printDebug("Failed to start canzero receive task");
	}
	TaskHandle_t heartbeatConsumerTaskHandle = nullptr;
	BaseType_t heartbeatConsumerStatus = xTaskCreate(
			canzero::heartbeat::consumer_entry, "cz_ht_con", 256, NULL,
			osPriorityNormal, &heartbeatConsumerTaskHandle);
	if(heartbeatConsumerStatus != pdPASS){
		printDebug("Failed to start canzero heartbeat consumer task");
	}

	TaskHandle_t heartbeatProducerTaskHandle = nullptr;
	BaseType_t heartbeatProducerStatus = xTaskCreate(
			canzero::heartbeat::producer_entry, "cz_ht_pro", 256, NULL,
			osPriorityNormal, &heartbeatProducerTaskHandle
			);
	if(heartbeatProducerStatus != pdPASS){
		printDebug("Failed to start canzero heartbeat producer task");
	}

	TaskHandle_t emcyTaskHandle = nullptr;
	BaseType_t emcyStatus = xTaskCreate(canzero::emergency::consumer_entry, "cz_emcy", 256, NULL,
			osPriorityHigh, &emcyTaskHandle);
	if(emcyStatus != pdPASS){
		Error_Handler();
	}


	while (true) {
		osDelay(osWaitForever);
	}
}

#ifdef __cplusplus
}
#endif

