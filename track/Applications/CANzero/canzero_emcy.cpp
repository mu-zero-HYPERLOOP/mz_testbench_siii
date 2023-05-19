/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This source file was generated from 'pod2023_gen.dbc' on 20:06:18 19.05.2023.
 * It contains the errors and warnings for the node 'Track'.
 *
 * Florian Keck
 * florian.keck@mu-zero.de
 * Copyright 2023, mu-zero HYPERLOOP e.V.
 */
#include "canzero_emcy.hpp"

/**************************************************************************
* CANzero Emergency: Errors & Warnings                                    *
*                                                                         *
* Call the get, set and clear function for errors and warnings.           *
*                                                                         *
* - These functions cannot be called from an ISR!                         *
* - Errors and warnings are signaled as bit flags to the EmergencyTask    *
*   using FreeRTOS Task Notifications.                                    *
* - We are not using the CMSIS-RTOS functions, since they reserve the     *
*   highest bit for signaling an error, but we need all 32 bits.          *
* - ulTaskNotifyValueClear() seems not to notifiy the task, this is why   *
*   in the clear() methods we notify the task explicitly.                 *
* - For get(), we do not use xTaskNotifyAndQuery() since this would       *
*   notify the emergency task, therefore we create the bool status flags. *
***************************************************************************/


extern osThreadId_t emergencyTaskHandle;


/**************************************************************************
* Functions to get, set and clear CANzero warnings                        *
***************************************************************************/
// Reset all warnings
void WARN_ALL_clear() {
    WARN_OtherWarning_clear();
}

// Warning: W0_OtherWarning
bool WARN_OtherWarning_status = false;    // Internal value

void WARN_OtherWarning_set() {
    // Only set warning when it is not set yet
    if(!WARN_OtherWarning_status) {

        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, WARN_OtherWarning_FLAG, eSetBits);

        // Update internal value
        WARN_OtherWarning_status = true;
    }
}

void WARN_OtherWarning_clear() {
    // Only clear warning when it is set
    if(WARN_OtherWarning_status) {
        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, WARN_OtherWarning_FLAG);

        // Update internal value
        WARN_OtherWarning_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task manually
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool WARN_OtherWarning_get() {
    // Return internal value
    return WARN_OtherWarning_status;
}




/**************************************************************************
* Functions to get, set and reset CANzero errors                          *
***************************************************************************/
// Reset all errors
void ERR_ALL_clear() {
    ERR_OtherError_clear();
}

// Error: E0_OtherError
bool ERR_OtherError_status = false;    // Internal value

void ERR_OtherError_set() {
    // Only set error when it is not set yet
    if(!ERR_OtherError_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_OtherError_FLAG, eSetBits);

        // Update internal value
        ERR_OtherError_status = true;
    }
}

void ERR_OtherError_clear() {
    // Only clear error when it is set
    if(ERR_OtherError_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_OtherError_FLAG);

        // Update internal value
        ERR_OtherError_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_OtherError_get() {
    // Return internal value
    return ERR_OtherError_status;
}


