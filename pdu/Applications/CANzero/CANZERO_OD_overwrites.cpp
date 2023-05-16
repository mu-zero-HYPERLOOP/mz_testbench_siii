/*
 * CANZERO_OD_overwrites.cpp
 *
 *  Created on: 27.01.2021
 *      Author: Felix
 */

#include "cmsis_os.h"
#include "canzero_od.hpp"
#include "FlashAccess.hpp"
#include "portmacro.h"
#include "TaskManager.hpp"
#include "canzero_defines.h"


// Get CANzero Node-ID of this node
uint8_t OD_NodeID_get() {
	return CANZERO_NODE_ID;
}

// Version of software components. Currently only the DBC version is implemented
uint16_t OD_ProtocolVersion_get() {
	return 1;
}
uint16_t OD_StackVersion_get() {
	return 1;
}
uint16_t OD_DbcVersion_get() {
	return can::CANzero_DBCVersion;
}

void OD_SendOdOnBootup_set(const uint8_t value){
    osMutexAcquire(mutex_OD_SendOdOnBootup, portMAX_DELAY);
    OD_SendOdOnBootup = value;
    flash_write(0x080E00000,value);
    osMutexRelease(mutex_OD_SendOdOnBootup);
}
uint8_t OD_SendOdOnBootup_get(){
    osMutexAcquire(mutex_OD_SendOdOnBootup, portMAX_DELAY);
    uint8_t value = flash_read(0x080E00000);
    osMutexRelease(mutex_OD_SendOdOnBootup);
    return value;
}

// Runtime in milliseconds, will overflow after 49 days.
uint32_t OD_runtime_get(){
    osMutexAcquire(mutex_OD_runtime, portMAX_DELAY);
    uint32_t value = portGET_RUN_TIME_COUNTER_VALUE();
    osMutexRelease(mutex_OD_runtime);
    return value;
}

// Report the status of the SDC input and output of this node
uint8_t OD_SdcIn_get() {
	return HAL_GPIO_ReadPin(SDC_IN_STATUS_GPIO_Port, SDC_IN_STATUS_Pin) == GPIO_PIN_SET;
}
uint8_t OD_SdcOut_get() {
	return HAL_GPIO_ReadPin(SDC_OUT_STATUS_GPIO_Port, SDC_OUT_STATUS_Pin) == GPIO_PIN_SET;
}

// 96-bit unique ID of the STM32 chip
uint64_t OD_ChipUID1_get() {
	return static_cast<uint64_t>(HAL_GetUIDw0()) + (static_cast<uint64_t>(HAL_GetUIDw1() & 0xFFFFull) << 32);
}
uint64_t OD_ChipUID2_get() {
	return static_cast<uint64_t>(HAL_GetUIDw1() & 0xFFFF0000ull) + (static_cast<uint64_t>(HAL_GetUIDw2()) << 16);
}

// Build date and time as integers
// Extern volatile variables are used so that the build time updates whenever main.cpp is recompiled.
extern volatile uint32_t BUILD_DATE;
extern volatile uint32_t BUILD_TIME;
uint32_t OD_BuildDate_get() {
	return BUILD_DATE;	// Format YYYYMMDD
}
uint32_t OD_BuildTime_get() {
	return BUILD_TIME;	// Format HHMMSS
}

// CAN1 and CAN2 internal values (e.g. error counters, status, ...)
uint8_t OD_CAN1_TxErrCnt_get() {
	uint32_t can_esr_reg = READ_REG(hcan1.Instance->ESR);
	uint8_t can_tec = static_cast<uint8_t>((can_esr_reg & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);		// 8-bit value
	return can_tec;
}
uint8_t OD_CAN1_RxErrCnt_get() {
	uint32_t can_esr_reg = READ_REG(hcan1.Instance->ESR);
	uint8_t can_rec = static_cast<uint8_t>((can_esr_reg & CAN_ESR_REC) >> CAN_ESR_REC_Pos);		// 8-bit value
	return can_rec;
}
uint32_t OD_CAN1_lastErrorCode_get() {
	return HAL_CAN_GetError(&hcan1);
}
uint8_t OD_CAN1_autoErrorRest_get() {
	// TODO
	return 0;
}
uint8_t OD_CAN1_Status_get() {
	return HAL_CAN_GetState(&hcan1);
}
uint8_t OD_CAN2_TxErrCnt_get() {
	uint32_t can_esr_reg = READ_REG(hcan2.Instance->ESR);
	uint8_t can_tec = static_cast<uint8_t>((can_esr_reg & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos);		// 8-bit value
	return can_tec;
}
uint8_t OD_CAN2_RxErrCnt_get() {
	uint32_t can_esr_reg = READ_REG(hcan2.Instance->ESR);
	uint8_t can_rec = static_cast<uint8_t>((can_esr_reg & CAN_ESR_REC) >> CAN_ESR_REC_Pos);		// 8-bit value
	return can_rec;
}
uint32_t OD_CAN2_lastErrorCode_get() {
	return HAL_CAN_GetError(&hcan2);
}
uint8_t OD_CAN2_autoErrorRest_get() {
	// TODO
	return 0;
}
uint8_t OD_CAN2_Status_get() {
	return HAL_CAN_GetState(&hcan2);
}

