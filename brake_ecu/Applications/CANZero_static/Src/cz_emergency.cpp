/*
 * cz_emergency_task.cpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */

#include "cz_emergency.hpp"
#include "dbc_parser.hpp"
#include "cz_weak.hpp"
#include <cinttypes>
#include "FreeRTOS.h"
#include "task.h"


TaskHandle_t emergencyTaskHandle;

void canzero::emergency::consumer_entry(void* argv){
	emergencyTaskHandle = xTaskGetCurrentTaskHandle();
	// Timeout for waiting for an emergency notification
	constexpr uint32_t EMERGENCY_WAIT_TIMEOUT_MS = 500;

	can::Message<can::messages::CANZERO_EMCY> emcyMessage;

	uint32_t emergencyBuffer = 0;
	uint32_t lastEmergencyBuffer = 0;

	while (true) {
		// Wait for notifications with a timeout without clearing flags
		xTaskNotifyWait(0, 0, &emergencyBuffer, pdMS_TO_TICKS(EMERGENCY_WAIT_TIMEOUT_MS));

		// When there was a change, call the user handler function
		if (emergencyBuffer != lastEmergencyBuffer) {
			//TODO: pass warning parameter.
			canzero::handle_emergency_warning();
			//canzero::handle_emergency_warning(emergencyBuffer, lastEmergencyBuffer);
		}

		// Send CAN message when there was a change or when there is an error / warning present
		if (emergencyBuffer != 0 || emergencyBuffer != lastEmergencyBuffer) {
			emcyMessage.intel = emergencyBuffer;
			emcyMessage.send();
		}

		lastEmergencyBuffer = emergencyBuffer;
	}
}


