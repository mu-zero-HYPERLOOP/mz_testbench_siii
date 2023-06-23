/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This source file was generated from 'pod2023_gen.dbc' on 03:09:35 22.06.2023.
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
    ERR_CPUOverTemp_clear();
    ERR_OverVolt_clear();
    ERR_UnderVolt_clear();
    ERR_InvalidPosition_clear();
    ERR_ReservoirOverTemp_clear();
    ERR_CLUHeartbeatMiss_clear();
    ERR_BECUHeartbeatMiss_clear();
    ERR_PDUHeartbeatMiss_clear();
    ERR_TelemetryHeartbeatMiss_clear();
    ERR_TitanOverTemp_clear();
    ERR_HyperionOverTemp_clear();
    ERR_TitanLowHp_clear();
    ERR_HyperionLowHp_clear();
    ERR_TitanLowCap_clear();
    ERR_HyperionLowCap_clear();
    ERR_EboxOverTemp_clear();
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


// Error: E1_CPUOverTemp
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


// Error: E3_UnderVolt
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


// Error: E4_InvalidPosition
bool ERR_InvalidPosition_status = false;    // Internal value

void ERR_InvalidPosition_set() {
    // Only set error when it is not set yet
    if(!ERR_InvalidPosition_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_InvalidPosition_FLAG, eSetBits);

        // Update internal value
        ERR_InvalidPosition_status = true;
    }
}

void ERR_InvalidPosition_clear() {
    // Only clear error when it is set
    if(ERR_InvalidPosition_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_InvalidPosition_FLAG);

        // Update internal value
        ERR_InvalidPosition_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_InvalidPosition_get() {
    // Return internal value
    return ERR_InvalidPosition_status;
}


// Error: E5_ReservoirOverTemp
bool ERR_ReservoirOverTemp_status = false;    // Internal value

void ERR_ReservoirOverTemp_set() {
    // Only set error when it is not set yet
    if(!ERR_ReservoirOverTemp_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_ReservoirOverTemp_FLAG, eSetBits);

        // Update internal value
        ERR_ReservoirOverTemp_status = true;
    }
}

void ERR_ReservoirOverTemp_clear() {
    // Only clear error when it is set
    if(ERR_ReservoirOverTemp_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_ReservoirOverTemp_FLAG);

        // Update internal value
        ERR_ReservoirOverTemp_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_ReservoirOverTemp_get() {
    // Return internal value
    return ERR_ReservoirOverTemp_status;
}


// Error: E6_CLUHeartbeatMiss
bool ERR_CLUHeartbeatMiss_status = false;    // Internal value

void ERR_CLUHeartbeatMiss_set() {
    // Only set error when it is not set yet
    if(!ERR_CLUHeartbeatMiss_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_CLUHeartbeatMiss_FLAG, eSetBits);

        // Update internal value
        ERR_CLUHeartbeatMiss_status = true;
    }
}

void ERR_CLUHeartbeatMiss_clear() {
    // Only clear error when it is set
    if(ERR_CLUHeartbeatMiss_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_CLUHeartbeatMiss_FLAG);

        // Update internal value
        ERR_CLUHeartbeatMiss_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_CLUHeartbeatMiss_get() {
    // Return internal value
    return ERR_CLUHeartbeatMiss_status;
}


// Error: E7_BECUHeartbeatMiss
bool ERR_BECUHeartbeatMiss_status = false;    // Internal value

void ERR_BECUHeartbeatMiss_set() {
    // Only set error when it is not set yet
    if(!ERR_BECUHeartbeatMiss_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_BECUHeartbeatMiss_FLAG, eSetBits);

        // Update internal value
        ERR_BECUHeartbeatMiss_status = true;
    }
}

void ERR_BECUHeartbeatMiss_clear() {
    // Only clear error when it is set
    if(ERR_BECUHeartbeatMiss_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_BECUHeartbeatMiss_FLAG);

        // Update internal value
        ERR_BECUHeartbeatMiss_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_BECUHeartbeatMiss_get() {
    // Return internal value
    return ERR_BECUHeartbeatMiss_status;
}


// Error: E8_PDUHeartbeatMiss
bool ERR_PDUHeartbeatMiss_status = false;    // Internal value

void ERR_PDUHeartbeatMiss_set() {
    // Only set error when it is not set yet
    if(!ERR_PDUHeartbeatMiss_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_PDUHeartbeatMiss_FLAG, eSetBits);

        // Update internal value
        ERR_PDUHeartbeatMiss_status = true;
    }
}

void ERR_PDUHeartbeatMiss_clear() {
    // Only clear error when it is set
    if(ERR_PDUHeartbeatMiss_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_PDUHeartbeatMiss_FLAG);

        // Update internal value
        ERR_PDUHeartbeatMiss_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_PDUHeartbeatMiss_get() {
    // Return internal value
    return ERR_PDUHeartbeatMiss_status;
}


// Error: E9_TelemetryHeartbeatMiss
bool ERR_TelemetryHeartbeatMiss_status = false;    // Internal value

void ERR_TelemetryHeartbeatMiss_set() {
    // Only set error when it is not set yet
    if(!ERR_TelemetryHeartbeatMiss_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_TelemetryHeartbeatMiss_FLAG, eSetBits);

        // Update internal value
        ERR_TelemetryHeartbeatMiss_status = true;
    }
}

void ERR_TelemetryHeartbeatMiss_clear() {
    // Only clear error when it is set
    if(ERR_TelemetryHeartbeatMiss_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_TelemetryHeartbeatMiss_FLAG);

        // Update internal value
        ERR_TelemetryHeartbeatMiss_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_TelemetryHeartbeatMiss_get() {
    // Return internal value
    return ERR_TelemetryHeartbeatMiss_status;
}


// Error: E10_TitanOverTemp
bool ERR_TitanOverTemp_status = false;    // Internal value

void ERR_TitanOverTemp_set() {
    // Only set error when it is not set yet
    if(!ERR_TitanOverTemp_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_TitanOverTemp_FLAG, eSetBits);

        // Update internal value
        ERR_TitanOverTemp_status = true;
    }
}

void ERR_TitanOverTemp_clear() {
    // Only clear error when it is set
    if(ERR_TitanOverTemp_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_TitanOverTemp_FLAG);

        // Update internal value
        ERR_TitanOverTemp_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_TitanOverTemp_get() {
    // Return internal value
    return ERR_TitanOverTemp_status;
}


// Error: E11_HyperionOverTemp
bool ERR_HyperionOverTemp_status = false;    // Internal value

void ERR_HyperionOverTemp_set() {
    // Only set error when it is not set yet
    if(!ERR_HyperionOverTemp_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_HyperionOverTemp_FLAG, eSetBits);

        // Update internal value
        ERR_HyperionOverTemp_status = true;
    }
}

void ERR_HyperionOverTemp_clear() {
    // Only clear error when it is set
    if(ERR_HyperionOverTemp_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_HyperionOverTemp_FLAG);

        // Update internal value
        ERR_HyperionOverTemp_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_HyperionOverTemp_get() {
    // Return internal value
    return ERR_HyperionOverTemp_status;
}


// Error: E12_TitanLowHp
bool ERR_TitanLowHp_status = false;    // Internal value

void ERR_TitanLowHp_set() {
    // Only set error when it is not set yet
    if(!ERR_TitanLowHp_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_TitanLowHp_FLAG, eSetBits);

        // Update internal value
        ERR_TitanLowHp_status = true;
    }
}

void ERR_TitanLowHp_clear() {
    // Only clear error when it is set
    if(ERR_TitanLowHp_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_TitanLowHp_FLAG);

        // Update internal value
        ERR_TitanLowHp_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_TitanLowHp_get() {
    // Return internal value
    return ERR_TitanLowHp_status;
}


// Error: E13_HyperionLowHp
bool ERR_HyperionLowHp_status = false;    // Internal value

void ERR_HyperionLowHp_set() {
    // Only set error when it is not set yet
    if(!ERR_HyperionLowHp_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_HyperionLowHp_FLAG, eSetBits);

        // Update internal value
        ERR_HyperionLowHp_status = true;
    }
}

void ERR_HyperionLowHp_clear() {
    // Only clear error when it is set
    if(ERR_HyperionLowHp_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_HyperionLowHp_FLAG);

        // Update internal value
        ERR_HyperionLowHp_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_HyperionLowHp_get() {
    // Return internal value
    return ERR_HyperionLowHp_status;
}


// Error: E14_TitanLowCap
bool ERR_TitanLowCap_status = false;    // Internal value

void ERR_TitanLowCap_set() {
    // Only set error when it is not set yet
    if(!ERR_TitanLowCap_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_TitanLowCap_FLAG, eSetBits);

        // Update internal value
        ERR_TitanLowCap_status = true;
    }
}

void ERR_TitanLowCap_clear() {
    // Only clear error when it is set
    if(ERR_TitanLowCap_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_TitanLowCap_FLAG);

        // Update internal value
        ERR_TitanLowCap_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_TitanLowCap_get() {
    // Return internal value
    return ERR_TitanLowCap_status;
}


// Error: E15_HyperionLowCap
bool ERR_HyperionLowCap_status = false;    // Internal value

void ERR_HyperionLowCap_set() {
    // Only set error when it is not set yet
    if(!ERR_HyperionLowCap_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_HyperionLowCap_FLAG, eSetBits);

        // Update internal value
        ERR_HyperionLowCap_status = true;
    }
}

void ERR_HyperionLowCap_clear() {
    // Only clear error when it is set
    if(ERR_HyperionLowCap_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_HyperionLowCap_FLAG);

        // Update internal value
        ERR_HyperionLowCap_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_HyperionLowCap_get() {
    // Return internal value
    return ERR_HyperionLowCap_status;
}


// Error: E16_EboxOverTemp
bool ERR_EboxOverTemp_status = false;    // Internal value

void ERR_EboxOverTemp_set() {
    // Only set error when it is not set yet
    if(!ERR_EboxOverTemp_status) {
            
        // Set bit and notify emergency task
        xTaskNotify((TaskHandle_t)emergencyTaskHandle, ERR_EboxOverTemp_FLAG, eSetBits);

        // Update internal value
        ERR_EboxOverTemp_status = true;
    }
}

void ERR_EboxOverTemp_clear() {
    // Only clear error when it is set
    if(ERR_EboxOverTemp_status) {

        // Reset bit
        (void)ulTaskNotifyValueClear((TaskHandle_t)emergencyTaskHandle, ERR_EboxOverTemp_FLAG);

        // Update internal value
        ERR_EboxOverTemp_status = false;

        // ulTaskNotifyValueClear() seems not to notify the task -> notify the task
        (void)xTaskNotify((TaskHandle_t)emergencyTaskHandle, 0, eNoAction);
    }
}

bool ERR_EboxOverTemp_get() {
    // Return internal value
    return ERR_EboxOverTemp_status;
}


