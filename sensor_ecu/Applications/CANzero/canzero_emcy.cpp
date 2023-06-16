/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This source file was generated from 'pod2023_gen.dbc' on 16:22:35 16.06.2023.
 * It contains the errors and warnings for the node 'SensorF'.
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
    WARN_StateMTransitionW_clear();
    WARN_encoderOORWarning_clear();
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


// Warning: W1_StateMTransitionW
bool WARN_StateMTransitionW_status = false;    // Internal value

void WARN_StateMTransitionW_set() {
    // Only set warning when it is not set yet
    if(!WARN_StateMTransitionW_status) {

        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, WARN_StateMTransitionW_FLAG, eSetBits);

        // Update internal value
        WARN_StateMTransitionW_status = true;
    }
}

void WARN_StateMTransitionW_clear() {
    // Only clear warning when it is set
    if(WARN_StateMTransitionW_status) {
        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, WARN_StateMTransitionW_FLAG);

        // Update internal value
        WARN_StateMTransitionW_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task manually
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool WARN_StateMTransitionW_get() {
    // Return internal value
    return WARN_StateMTransitionW_status;
}


// Warning: W2_encoderOORWarning
bool WARN_encoderOORWarning_status = false;    // Internal value

void WARN_encoderOORWarning_set() {
    // Only set warning when it is not set yet
    if(!WARN_encoderOORWarning_status) {

        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, WARN_encoderOORWarning_FLAG, eSetBits);

        // Update internal value
        WARN_encoderOORWarning_status = true;
    }
}

void WARN_encoderOORWarning_clear() {
    // Only clear warning when it is set
    if(WARN_encoderOORWarning_status) {
        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, WARN_encoderOORWarning_FLAG);

        // Update internal value
        WARN_encoderOORWarning_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task manually
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool WARN_encoderOORWarning_get() {
    // Return internal value
    return WARN_encoderOORWarning_status;
}




/**************************************************************************
* Functions to get, set and reset CANzero errors                          *
***************************************************************************/
// Reset all errors
void ERR_ALL_clear() {
    ERR_OtherError_clear();
    ERR_StateMTransitionE_clear();
    ERR_BrakeFTimeout_clear();
    ERR_BrakeRTimeout_clear();
    ERR_PDUTimeout_clear();
    ERR_HVCUTimeout_clear();
    ERR_SensorRTimeout_clear();
    ERR_TelemetryTimeout_clear();
    ERR_NodeErrorFlag_clear();
    ERR_SWError_clear();
    ERR_TelemEmergency_clear();
    ERR_encoderError_clear();
    ERR_encoderSpeedError_clear();
    ERR_fiducialHighOffset_clear();
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


// Error: E1_StateMTransitionE
bool ERR_StateMTransitionE_status = false;    // Internal value

void ERR_StateMTransitionE_set() {
    // Only set error when it is not set yet
    if(!ERR_StateMTransitionE_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_StateMTransitionE_FLAG, eSetBits);

        // Update internal value
        ERR_StateMTransitionE_status = true;
    }
}

void ERR_StateMTransitionE_clear() {
    // Only clear error when it is set
    if(ERR_StateMTransitionE_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_StateMTransitionE_FLAG);

        // Update internal value
        ERR_StateMTransitionE_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_StateMTransitionE_get() {
    // Return internal value
    return ERR_StateMTransitionE_status;
}


// Error: E2_BrakeFTimeout
bool ERR_BrakeFTimeout_status = false;    // Internal value

void ERR_BrakeFTimeout_set() {
    // Only set error when it is not set yet
    if(!ERR_BrakeFTimeout_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_BrakeFTimeout_FLAG, eSetBits);

        // Update internal value
        ERR_BrakeFTimeout_status = true;
    }
}

void ERR_BrakeFTimeout_clear() {
    // Only clear error when it is set
    if(ERR_BrakeFTimeout_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_BrakeFTimeout_FLAG);

        // Update internal value
        ERR_BrakeFTimeout_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_BrakeFTimeout_get() {
    // Return internal value
    return ERR_BrakeFTimeout_status;
}


// Error: E3_BrakeRTimeout
bool ERR_BrakeRTimeout_status = false;    // Internal value

void ERR_BrakeRTimeout_set() {
    // Only set error when it is not set yet
    if(!ERR_BrakeRTimeout_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_BrakeRTimeout_FLAG, eSetBits);

        // Update internal value
        ERR_BrakeRTimeout_status = true;
    }
}

void ERR_BrakeRTimeout_clear() {
    // Only clear error when it is set
    if(ERR_BrakeRTimeout_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_BrakeRTimeout_FLAG);

        // Update internal value
        ERR_BrakeRTimeout_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_BrakeRTimeout_get() {
    // Return internal value
    return ERR_BrakeRTimeout_status;
}


// Error: E4_PDUTimeout
bool ERR_PDUTimeout_status = false;    // Internal value

void ERR_PDUTimeout_set() {
    // Only set error when it is not set yet
    if(!ERR_PDUTimeout_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_PDUTimeout_FLAG, eSetBits);

        // Update internal value
        ERR_PDUTimeout_status = true;
    }
}

void ERR_PDUTimeout_clear() {
    // Only clear error when it is set
    if(ERR_PDUTimeout_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_PDUTimeout_FLAG);

        // Update internal value
        ERR_PDUTimeout_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_PDUTimeout_get() {
    // Return internal value
    return ERR_PDUTimeout_status;
}


// Error: E5_HVCUTimeout
bool ERR_HVCUTimeout_status = false;    // Internal value

void ERR_HVCUTimeout_set() {
    // Only set error when it is not set yet
    if(!ERR_HVCUTimeout_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_HVCUTimeout_FLAG, eSetBits);

        // Update internal value
        ERR_HVCUTimeout_status = true;
    }
}

void ERR_HVCUTimeout_clear() {
    // Only clear error when it is set
    if(ERR_HVCUTimeout_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_HVCUTimeout_FLAG);

        // Update internal value
        ERR_HVCUTimeout_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_HVCUTimeout_get() {
    // Return internal value
    return ERR_HVCUTimeout_status;
}


// Error: E6_SensorRTimeout
bool ERR_SensorRTimeout_status = false;    // Internal value

void ERR_SensorRTimeout_set() {
    // Only set error when it is not set yet
    if(!ERR_SensorRTimeout_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_SensorRTimeout_FLAG, eSetBits);

        // Update internal value
        ERR_SensorRTimeout_status = true;
    }
}

void ERR_SensorRTimeout_clear() {
    // Only clear error when it is set
    if(ERR_SensorRTimeout_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_SensorRTimeout_FLAG);

        // Update internal value
        ERR_SensorRTimeout_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_SensorRTimeout_get() {
    // Return internal value
    return ERR_SensorRTimeout_status;
}


// Error: E7_TelemetryTimeout
bool ERR_TelemetryTimeout_status = false;    // Internal value

void ERR_TelemetryTimeout_set() {
    // Only set error when it is not set yet
    if(!ERR_TelemetryTimeout_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_TelemetryTimeout_FLAG, eSetBits);

        // Update internal value
        ERR_TelemetryTimeout_status = true;
    }
}

void ERR_TelemetryTimeout_clear() {
    // Only clear error when it is set
    if(ERR_TelemetryTimeout_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_TelemetryTimeout_FLAG);

        // Update internal value
        ERR_TelemetryTimeout_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_TelemetryTimeout_get() {
    // Return internal value
    return ERR_TelemetryTimeout_status;
}


// Error: E8_NodeErrorFlag
bool ERR_NodeErrorFlag_status = false;    // Internal value

void ERR_NodeErrorFlag_set() {
    // Only set error when it is not set yet
    if(!ERR_NodeErrorFlag_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_NodeErrorFlag_FLAG, eSetBits);

        // Update internal value
        ERR_NodeErrorFlag_status = true;
    }
}

void ERR_NodeErrorFlag_clear() {
    // Only clear error when it is set
    if(ERR_NodeErrorFlag_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_NodeErrorFlag_FLAG);

        // Update internal value
        ERR_NodeErrorFlag_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_NodeErrorFlag_get() {
    // Return internal value
    return ERR_NodeErrorFlag_status;
}


// Error: E9_SWError
bool ERR_SWError_status = false;    // Internal value

void ERR_SWError_set() {
    // Only set error when it is not set yet
    if(!ERR_SWError_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_SWError_FLAG, eSetBits);

        // Update internal value
        ERR_SWError_status = true;
    }
}

void ERR_SWError_clear() {
    // Only clear error when it is set
    if(ERR_SWError_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_SWError_FLAG);

        // Update internal value
        ERR_SWError_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_SWError_get() {
    // Return internal value
    return ERR_SWError_status;
}


// Error: E10_TelemEmergency
bool ERR_TelemEmergency_status = false;    // Internal value

void ERR_TelemEmergency_set() {
    // Only set error when it is not set yet
    if(!ERR_TelemEmergency_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_TelemEmergency_FLAG, eSetBits);

        // Update internal value
        ERR_TelemEmergency_status = true;
    }
}

void ERR_TelemEmergency_clear() {
    // Only clear error when it is set
    if(ERR_TelemEmergency_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_TelemEmergency_FLAG);

        // Update internal value
        ERR_TelemEmergency_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_TelemEmergency_get() {
    // Return internal value
    return ERR_TelemEmergency_status;
}


// Error: E12_encoderError
bool ERR_encoderError_status = false;    // Internal value

void ERR_encoderError_set() {
    // Only set error when it is not set yet
    if(!ERR_encoderError_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_encoderError_FLAG, eSetBits);

        // Update internal value
        ERR_encoderError_status = true;
    }
}

void ERR_encoderError_clear() {
    // Only clear error when it is set
    if(ERR_encoderError_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_encoderError_FLAG);

        // Update internal value
        ERR_encoderError_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_encoderError_get() {
    // Return internal value
    return ERR_encoderError_status;
}


// Error: E13_encoderSpeedError
bool ERR_encoderSpeedError_status = false;    // Internal value

void ERR_encoderSpeedError_set() {
    // Only set error when it is not set yet
    if(!ERR_encoderSpeedError_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_encoderSpeedError_FLAG, eSetBits);

        // Update internal value
        ERR_encoderSpeedError_status = true;
    }
}

void ERR_encoderSpeedError_clear() {
    // Only clear error when it is set
    if(ERR_encoderSpeedError_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_encoderSpeedError_FLAG);

        // Update internal value
        ERR_encoderSpeedError_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_encoderSpeedError_get() {
    // Return internal value
    return ERR_encoderSpeedError_status;
}


// Error: E14_fiducialHighOffset
bool ERR_fiducialHighOffset_status = false;    // Internal value

void ERR_fiducialHighOffset_set() {
    // Only set error when it is not set yet
    if(!ERR_fiducialHighOffset_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_fiducialHighOffset_FLAG, eSetBits);

        // Update internal value
        ERR_fiducialHighOffset_status = true;
    }
}

void ERR_fiducialHighOffset_clear() {
    // Only clear error when it is set
    if(ERR_fiducialHighOffset_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_fiducialHighOffset_FLAG);

        // Update internal value
        ERR_fiducialHighOffset_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_fiducialHighOffset_get() {
    // Return internal value
    return ERR_fiducialHighOffset_status;
}


