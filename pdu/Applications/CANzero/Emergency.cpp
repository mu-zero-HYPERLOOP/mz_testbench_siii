/**
 * @file Emergency.cpp
 *
 * @date 08.12.2020
 * @author: Felix
 */
/**
 * @addtogroup CANzero
 * @{
 */
#include <Emergency.hpp>
#include <cz_interface.hpp>
#include "canzero_defines.h"

/**< singleton object of the Emergency class*/
Emergency *Emergency::em = nullptr;

Emergency::Emergency() {
}


/// This function gets called whenever a warning or error is set or cleared
/// If you want to use it, overwrite it (without the weak attribute) in any other file.
void __attribute__((weak)) Emergency::handleEmergency(uint32_t emergencyBuffer, uint32_t lastEmergencyBuffer) {
	/** Some usage examples below

	// Check for any error
	if(emergencyBuffer & ERR_ANY_FLAG) {
		// We have at least one error -> e.g. open SDC
	} else {
		// No error
	}

	// Check for a specific error
	if(emergencyBuffer & ERR_OtherError_FLAG) {
		// Error OtherError is present
	} else {
		// Error OtherError no present
	}

	*/
}

/**
 * @brief Emergency Task
 * This Task checks new error or warnings, processes them with handleEmergency and
 * sends them on the CAN bus with can::Message::send which puts the message in cz_interface::cz_send
 *
 * @param params: unused
 */
void Emergency::waitForEmergency(void *params) {
	// Timeout for waiting for an emergency notification
	constexpr uint32_t EMERGENCY_WAIT_TIMEOUT_MS = 500;

	can::Message<can::messages::CANZERO_EMCY> emcyMessage;

	uint32_t emergencyBuffer = 0;
	uint32_t lastEmergencyBuffer = 0;

	while (1) {
		// Wait for notifications with a timeout without clearing flags
		xTaskNotifyWait(0, 0, &emergencyBuffer, pdMS_TO_TICKS(EMERGENCY_WAIT_TIMEOUT_MS));

		// When there was a change, call the user handler function
		if (emergencyBuffer != lastEmergencyBuffer) {
			handleEmergency(emergencyBuffer, lastEmergencyBuffer);
		}

		// Send CAN message when there was a change or when there is an error / warning present
		if (emergencyBuffer != 0 || emergencyBuffer != lastEmergencyBuffer) {
			emcyMessage.intel = emergencyBuffer;
			emcyMessage.send();
		}

		lastEmergencyBuffer = emergencyBuffer;
	}
}


Emergency::~Emergency() {
}

/**
 * @brief create and get the singleton of the Emergency class
 *
 * @return em Emergnecy singleton
 */
Emergency* Emergency::getEmergencyInstance() {

	if (em == nullptr) {
		em = new Emergency();
	}
	return em;
}
/**
 * @}
 */
