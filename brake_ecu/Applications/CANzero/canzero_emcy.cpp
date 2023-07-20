/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This source file was generated from 'pod2023_gen.dbc' on 20:21:44 05.07.2023.
 * It contains the errors and warnings for the node 'BrakeF'.
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
    WARN_highPressureActingChamber_clear();
    WARN_highPressureRetractingChamber_clear();
    WARN_enableWithAnError_clear();
    WARN_externalError_clear();
}

// Warning: W0_highPressureActingChamber
bool WARN_highPressureActingChamber_status = false;    // Internal value

void WARN_highPressureActingChamber_set() {
    // Only set warning when it is not set yet
    if(!WARN_highPressureActingChamber_status) {

        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, WARN_highPressureActingChamber_FLAG, eSetBits);

        // Update internal value
        WARN_highPressureActingChamber_status = true;
    }
}

void WARN_highPressureActingChamber_clear() {
    // Only clear warning when it is set
    if(WARN_highPressureActingChamber_status) {
        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, WARN_highPressureActingChamber_FLAG);

        // Update internal value
        WARN_highPressureActingChamber_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task manually
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool WARN_highPressureActingChamber_get() {
    // Return internal value
    return WARN_highPressureActingChamber_status;
}


// Warning: W1_highPressureRetractingChamber
bool WARN_highPressureRetractingChamber_status = false;    // Internal value

void WARN_highPressureRetractingChamber_set() {
    // Only set warning when it is not set yet
    if(!WARN_highPressureRetractingChamber_status) {

        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, WARN_highPressureRetractingChamber_FLAG, eSetBits);

        // Update internal value
        WARN_highPressureRetractingChamber_status = true;
    }
}

void WARN_highPressureRetractingChamber_clear() {
    // Only clear warning when it is set
    if(WARN_highPressureRetractingChamber_status) {
        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, WARN_highPressureRetractingChamber_FLAG);

        // Update internal value
        WARN_highPressureRetractingChamber_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task manually
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool WARN_highPressureRetractingChamber_get() {
    // Return internal value
    return WARN_highPressureRetractingChamber_status;
}


// Warning: W2_enableWithAnError
bool WARN_enableWithAnError_status = false;    // Internal value

void WARN_enableWithAnError_set() {
    // Only set warning when it is not set yet
    if(!WARN_enableWithAnError_status) {

        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, WARN_enableWithAnError_FLAG, eSetBits);

        // Update internal value
        WARN_enableWithAnError_status = true;
    }
}

void WARN_enableWithAnError_clear() {
    // Only clear warning when it is set
    if(WARN_enableWithAnError_status) {
        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, WARN_enableWithAnError_FLAG);

        // Update internal value
        WARN_enableWithAnError_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task manually
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool WARN_enableWithAnError_get() {
    // Return internal value
    return WARN_enableWithAnError_status;
}


// Warning: W3_externalError
bool WARN_externalError_status = false;    // Internal value

void WARN_externalError_set() {
    // Only set warning when it is not set yet
    if(!WARN_externalError_status) {

        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, WARN_externalError_FLAG, eSetBits);

        // Update internal value
        WARN_externalError_status = true;
    }
}

void WARN_externalError_clear() {
    // Only clear warning when it is set
    if(WARN_externalError_status) {
        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, WARN_externalError_FLAG);

        // Update internal value
        WARN_externalError_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task manually
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool WARN_externalError_get() {
    // Return internal value
    return WARN_externalError_status;
}




/**************************************************************************
* Functions to get, set and reset CANzero errors                          *
***************************************************************************/
// Reset all errors
void ERR_ALL_clear() {
    ERR_CPUOverTemp_clear();
    ERR_UnderVolt_clear();
    ERR_OverVolt_clear();
    ERR_IntakeOverPressure_clear();
    ERR_IntakeUnderPressure_clear();
    ERR_OuttakeOverPressure_clear();
    ERR_OuttakeUnderPressure_clear();
}

// Error: E0_CPUOverTemp
bool ERR_CPUOverTemp_status = false;    // Internal value

void ERR_CPUOverTemp_set() {
    // Only set error when it is not set yet
    if(!ERR_CPUOverTemp_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_CPUOverTemp_FLAG, eSetBits);

        // Update internal value
        ERR_CPUOverTemp_status = true;
    }
}

void ERR_CPUOverTemp_clear() {
    // Only clear error when it is set
    if(ERR_CPUOverTemp_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_CPUOverTemp_FLAG);

        // Update internal value
        ERR_CPUOverTemp_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_CPUOverTemp_get() {
    // Return internal value
    return ERR_CPUOverTemp_status;
}


// Error: E1_UnderVolt
bool ERR_UnderVolt_status = false;    // Internal value

void ERR_UnderVolt_set() {
    // Only set error when it is not set yet
    if(!ERR_UnderVolt_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_UnderVolt_FLAG, eSetBits);

        // Update internal value
        ERR_UnderVolt_status = true;
    }
}

void ERR_UnderVolt_clear() {
    // Only clear error when it is set
    if(ERR_UnderVolt_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_UnderVolt_FLAG);

        // Update internal value
        ERR_UnderVolt_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_UnderVolt_get() {
    // Return internal value
    return ERR_UnderVolt_status;
}


// Error: E2_OverVolt
bool ERR_OverVolt_status = false;    // Internal value

void ERR_OverVolt_set() {
    // Only set error when it is not set yet
    if(!ERR_OverVolt_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_OverVolt_FLAG, eSetBits);

        // Update internal value
        ERR_OverVolt_status = true;
    }
}

void ERR_OverVolt_clear() {
    // Only clear error when it is set
    if(ERR_OverVolt_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_OverVolt_FLAG);

        // Update internal value
        ERR_OverVolt_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_OverVolt_get() {
    // Return internal value
    return ERR_OverVolt_status;
}


// Error: E3_IntakeOverPressure
bool ERR_IntakeOverPressure_status = false;    // Internal value

void ERR_IntakeOverPressure_set() {
    // Only set error when it is not set yet
    if(!ERR_IntakeOverPressure_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_IntakeOverPressure_FLAG, eSetBits);

        // Update internal value
        ERR_IntakeOverPressure_status = true;
    }
}

void ERR_IntakeOverPressure_clear() {
    // Only clear error when it is set
    if(ERR_IntakeOverPressure_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_IntakeOverPressure_FLAG);

        // Update internal value
        ERR_IntakeOverPressure_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_IntakeOverPressure_get() {
    // Return internal value
    return ERR_IntakeOverPressure_status;
}


// Error: E4_IntakeUnderPressure
bool ERR_IntakeUnderPressure_status = false;    // Internal value

void ERR_IntakeUnderPressure_set() {
    // Only set error when it is not set yet
    if(!ERR_IntakeUnderPressure_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_IntakeUnderPressure_FLAG, eSetBits);

        // Update internal value
        ERR_IntakeUnderPressure_status = true;
    }
}

void ERR_IntakeUnderPressure_clear() {
    // Only clear error when it is set
    if(ERR_IntakeUnderPressure_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_IntakeUnderPressure_FLAG);

        // Update internal value
        ERR_IntakeUnderPressure_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_IntakeUnderPressure_get() {
    // Return internal value
    return ERR_IntakeUnderPressure_status;
}


// Error: E5_OuttakeOverPressure
bool ERR_OuttakeOverPressure_status = false;    // Internal value

void ERR_OuttakeOverPressure_set() {
    // Only set error when it is not set yet
    if(!ERR_OuttakeOverPressure_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_OuttakeOverPressure_FLAG, eSetBits);

        // Update internal value
        ERR_OuttakeOverPressure_status = true;
    }
}

void ERR_OuttakeOverPressure_clear() {
    // Only clear error when it is set
    if(ERR_OuttakeOverPressure_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_OuttakeOverPressure_FLAG);

        // Update internal value
        ERR_OuttakeOverPressure_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_OuttakeOverPressure_get() {
    // Return internal value
    return ERR_OuttakeOverPressure_status;
}


// Error: E6_OuttakeUnderPressure
bool ERR_OuttakeUnderPressure_status = false;    // Internal value

void ERR_OuttakeUnderPressure_set() {
    // Only set error when it is not set yet
    if(!ERR_OuttakeUnderPressure_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_OuttakeUnderPressure_FLAG, eSetBits);

        // Update internal value
        ERR_OuttakeUnderPressure_status = true;
    }
}

void ERR_OuttakeUnderPressure_clear() {
    // Only clear error when it is set
    if(ERR_OuttakeUnderPressure_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_OuttakeUnderPressure_FLAG);

        // Update internal value
        ERR_OuttakeUnderPressure_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_OuttakeUnderPressure_get() {
    // Return internal value
    return ERR_OuttakeUnderPressure_status;
}


