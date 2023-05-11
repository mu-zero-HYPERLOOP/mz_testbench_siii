/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This header file was generated from 'pod2023_gen.dbc'.
 * It contains the object dictionary for the node 'SensorF'.
 *
 * Florian Keck
 * florian.keck@mu-zero.de
 * Copyright 2023, mu-zero HYPERLOOP e.V.
 */
#ifndef CANZERO_SensorF_OD_HPP
#define CANZERO_SensorF_OD_HPP

#pragma once

#include "cz_interface.hpp"
#include "dbc_parser.hpp"


/**************************************************************************
* Declaration of all OD variables.                                        *
***************************************************************************/
extern volatile uint8_t  OD_NodeID;
extern volatile uint8_t  OD_NodeStatus;
extern volatile uint16_t OD_ProtocolVersion;
extern volatile uint16_t OD_StackVersion;
extern volatile uint16_t OD_DbcVersion;
extern volatile uint16_t OD_HeartbeatInterval;
extern volatile uint8_t  OD_SendOdOnBootup;
extern volatile uint16_t OD_OdEntrySendInterval;
extern volatile float    OD_CpuUsage;
extern volatile uint32_t OD_MemFree;
extern volatile float    OD_BoardTemp;
extern volatile float    OD_InputVoltage;
extern volatile uint32_t OD_runtime;
extern volatile uint8_t  OD_SdcIn;
extern volatile uint8_t  OD_SdcOut;
extern volatile uint64_t OD_ChipUID1;
extern volatile uint64_t OD_ChipUID2;
extern volatile uint32_t OD_BuildDate;
extern volatile uint32_t OD_BuildTime;
extern volatile uint8_t  OD_CAN1_TxErrCnt;
extern volatile uint8_t  OD_CAN1_RxErrCnt;
extern volatile uint32_t OD_CAN1_lastErrorCode;
extern volatile uint8_t  OD_CAN1_autoErrorReset;
extern volatile uint16_t OD_CAN1_Baudrate;
extern volatile uint8_t  OD_CAN1_Status;
extern volatile uint32_t OD_CAN1_DiscardedTxMessages;
extern volatile uint8_t  OD_CAN1_ErrorStatus;
extern volatile uint32_t OD_CAN1_DelayedTxMessages;
extern volatile uint8_t  OD_CAN2_TxErrCnt;
extern volatile uint8_t  OD_CAN2_RxErrCnt;
extern volatile uint32_t OD_CAN2_lastErrorCode;
extern volatile uint8_t  OD_CAN2_autoErrorReset;
extern volatile uint16_t OD_CAN2_Baudrate;
extern volatile uint8_t  OD_CAN2_Status;
extern volatile uint32_t OD_CAN2_DiscardedTxMessages;
extern volatile uint8_t  OD_CAN2_ErrorStatus;
extern volatile uint32_t OD_CAN2_DelayedTxMessages;
extern volatile float    OD_samplingInterval;
extern volatile uint8_t  OD_TelemetryCommands;
extern volatile uint8_t  OD_StateMachineInterval;
extern volatile uint8_t  OD_StateMachineActivate;
extern volatile uint8_t  OD_HVBatteryMode;
extern volatile float    OD_EncoderWheelDiameter;
extern volatile uint8_t  OD_EncoderResetPosition;
extern volatile uint8_t  OD_SetReset;
extern volatile uint8_t  OD_IMU_number;
extern volatile float    OD_IMU1_Temperature;
extern volatile float    OD_IMU2_Temperature;
extern volatile float    OD_IMU3_Temperature;
extern volatile float    OD_IMU_AccelX;
extern volatile float    OD_IMU_AccelY;
extern volatile float    OD_IMU_AccelZ;
extern volatile float    OD_IMU_GyroX;
extern volatile float    OD_IMU_GyroY;
extern volatile float    OD_IMU_GyroZ;
extern volatile float    OD_CoolingPressure;
extern volatile uint16_t OD_FiducialRightCounter;
extern volatile uint16_t OD_FiducialLeftCounter;
extern volatile float    OD_Position;
extern volatile float    OD_Velocity;

/**************************************************************************
* Semaphores for access to OD values                                      *
***************************************************************************/
extern osMutexId_t mutex_OD_NodeID;
extern osMutexId_t mutex_OD_NodeStatus;
extern osMutexId_t mutex_OD_ProtocolVersion;
extern osMutexId_t mutex_OD_StackVersion;
extern osMutexId_t mutex_OD_DbcVersion;
extern osMutexId_t mutex_OD_HeartbeatInterval;
extern osMutexId_t mutex_OD_SendOdOnBootup;
extern osMutexId_t mutex_OD_OdEntrySendInterval;
extern osMutexId_t mutex_OD_CpuUsage;
extern osMutexId_t mutex_OD_MemFree;
extern osMutexId_t mutex_OD_BoardTemp;
extern osMutexId_t mutex_OD_InputVoltage;
extern osMutexId_t mutex_OD_runtime;
extern osMutexId_t mutex_OD_SdcIn;
extern osMutexId_t mutex_OD_SdcOut;
extern osMutexId_t mutex_OD_ChipUID1;
extern osMutexId_t mutex_OD_ChipUID2;
extern osMutexId_t mutex_OD_BuildDate;
extern osMutexId_t mutex_OD_BuildTime;
extern osMutexId_t mutex_OD_CAN1_TxErrCnt;
extern osMutexId_t mutex_OD_CAN1_RxErrCnt;
extern osMutexId_t mutex_OD_CAN1_lastErrorCode;
extern osMutexId_t mutex_OD_CAN1_autoErrorReset;
extern osMutexId_t mutex_OD_CAN1_Baudrate;
extern osMutexId_t mutex_OD_CAN1_Status;
extern osMutexId_t mutex_OD_CAN1_DiscardedTxMessages;
extern osMutexId_t mutex_OD_CAN1_ErrorStatus;
extern osMutexId_t mutex_OD_CAN1_DelayedTxMessages;
extern osMutexId_t mutex_OD_CAN2_TxErrCnt;
extern osMutexId_t mutex_OD_CAN2_RxErrCnt;
extern osMutexId_t mutex_OD_CAN2_lastErrorCode;
extern osMutexId_t mutex_OD_CAN2_autoErrorReset;
extern osMutexId_t mutex_OD_CAN2_Baudrate;
extern osMutexId_t mutex_OD_CAN2_Status;
extern osMutexId_t mutex_OD_CAN2_DiscardedTxMessages;
extern osMutexId_t mutex_OD_CAN2_ErrorStatus;
extern osMutexId_t mutex_OD_CAN2_DelayedTxMessages;
extern osMutexId_t mutex_OD_samplingInterval;
extern osMutexId_t mutex_OD_TelemetryCommands;
extern osMutexId_t mutex_OD_StateMachineInterval;
extern osMutexId_t mutex_OD_StateMachineActivate;
extern osMutexId_t mutex_OD_HVBatteryMode;
extern osMutexId_t mutex_OD_EncoderWheelDiameter;
extern osMutexId_t mutex_OD_EncoderResetPosition;
extern osMutexId_t mutex_OD_SetReset;
extern osMutexId_t mutex_OD_IMU_number;
extern osMutexId_t mutex_OD_IMU1_Temperature;
extern osMutexId_t mutex_OD_IMU2_Temperature;
extern osMutexId_t mutex_OD_IMU3_Temperature;
extern osMutexId_t mutex_OD_IMU_AccelX;
extern osMutexId_t mutex_OD_IMU_AccelY;
extern osMutexId_t mutex_OD_IMU_AccelZ;
extern osMutexId_t mutex_OD_IMU_GyroX;
extern osMutexId_t mutex_OD_IMU_GyroY;
extern osMutexId_t mutex_OD_IMU_GyroZ;
extern osMutexId_t mutex_OD_CoolingPressure;
extern osMutexId_t mutex_OD_FiducialRightCounter;
extern osMutexId_t mutex_OD_FiducialLeftCounter;
extern osMutexId_t mutex_OD_Position;
extern osMutexId_t mutex_OD_Velocity;


/**************************************************************************
* Function to handle a SDO Request                                        *
***************************************************************************/
void handleSDORequestDownload(const RxMessage& rxMsgSdoReq);
void handleSDORequestDownloadBySDOID(const uint16_t sdoId);
void handleSDORequestUpload(const RxMessage& rxMsgSdoReq);


/**************************************************************************
* Functions for setting and getting an OD entry.                          *
* They are define weak, so can be overwritten in an external file.        *
* They can be overwritten, e.g. to read a value directly from hardware    *
* or to trigger another function (e.g. enter debug mode).                 *
* ATTENTION: Then the threadsafe access has to be handled by the user,    *
* e.g with using the provided mutex.                                      *
***************************************************************************/
uint8_t OD_NodeID_get();
void OD_NodeID_set(const uint8_t value);

uint8_t OD_NodeStatus_get();
void OD_NodeStatus_set(const uint8_t value);

uint16_t OD_ProtocolVersion_get();
void OD_ProtocolVersion_set(const uint16_t value);

uint16_t OD_StackVersion_get();
void OD_StackVersion_set(const uint16_t value);

uint16_t OD_DbcVersion_get();
void OD_DbcVersion_set(const uint16_t value);

uint16_t OD_HeartbeatInterval_get();
void OD_HeartbeatInterval_set(const uint16_t value);

uint8_t OD_SendOdOnBootup_get();
void OD_SendOdOnBootup_set(const uint8_t value);

uint16_t OD_OdEntrySendInterval_get();
void OD_OdEntrySendInterval_set(const uint16_t value);

float OD_CpuUsage_get();
void OD_CpuUsage_set(const float value);

uint32_t OD_MemFree_get();
void OD_MemFree_set(const uint32_t value);

float OD_BoardTemp_get();
void OD_BoardTemp_set(const float value);

float OD_InputVoltage_get();
void OD_InputVoltage_set(const float value);

uint32_t OD_runtime_get();
void OD_runtime_set(const uint32_t value);

uint8_t OD_SdcIn_get();
void OD_SdcIn_set(const uint8_t value);

uint8_t OD_SdcOut_get();
void OD_SdcOut_set(const uint8_t value);

uint64_t OD_ChipUID1_get();
void OD_ChipUID1_set(const uint64_t value);

uint64_t OD_ChipUID2_get();
void OD_ChipUID2_set(const uint64_t value);

uint32_t OD_BuildDate_get();
void OD_BuildDate_set(const uint32_t value);

uint32_t OD_BuildTime_get();
void OD_BuildTime_set(const uint32_t value);

uint8_t OD_CAN1_TxErrCnt_get();
void OD_CAN1_TxErrCnt_set(const uint8_t value);

uint8_t OD_CAN1_RxErrCnt_get();
void OD_CAN1_RxErrCnt_set(const uint8_t value);

uint32_t OD_CAN1_lastErrorCode_get();
void OD_CAN1_lastErrorCode_set(const uint32_t value);

uint8_t OD_CAN1_autoErrorReset_get();
void OD_CAN1_autoErrorReset_set(const uint8_t value);

uint16_t OD_CAN1_Baudrate_get();
void OD_CAN1_Baudrate_set(const uint16_t value);

uint8_t OD_CAN1_Status_get();
void OD_CAN1_Status_set(const uint8_t value);

uint32_t OD_CAN1_DiscardedTxMessages_get();
void OD_CAN1_DiscardedTxMessages_set(const uint32_t value);

uint8_t OD_CAN1_ErrorStatus_get();
void OD_CAN1_ErrorStatus_set(const uint8_t value);

uint32_t OD_CAN1_DelayedTxMessages_get();
void OD_CAN1_DelayedTxMessages_set(const uint32_t value);

uint8_t OD_CAN2_TxErrCnt_get();
void OD_CAN2_TxErrCnt_set(const uint8_t value);

uint8_t OD_CAN2_RxErrCnt_get();
void OD_CAN2_RxErrCnt_set(const uint8_t value);

uint32_t OD_CAN2_lastErrorCode_get();
void OD_CAN2_lastErrorCode_set(const uint32_t value);

uint8_t OD_CAN2_autoErrorReset_get();
void OD_CAN2_autoErrorReset_set(const uint8_t value);

uint16_t OD_CAN2_Baudrate_get();
void OD_CAN2_Baudrate_set(const uint16_t value);

uint8_t OD_CAN2_Status_get();
void OD_CAN2_Status_set(const uint8_t value);

uint32_t OD_CAN2_DiscardedTxMessages_get();
void OD_CAN2_DiscardedTxMessages_set(const uint32_t value);

uint8_t OD_CAN2_ErrorStatus_get();
void OD_CAN2_ErrorStatus_set(const uint8_t value);

uint32_t OD_CAN2_DelayedTxMessages_get();
void OD_CAN2_DelayedTxMessages_set(const uint32_t value);

float OD_samplingInterval_get();
void OD_samplingInterval_set(const float value);

uint8_t OD_TelemetryCommands_get();
void OD_TelemetryCommands_set(const uint8_t value);

uint8_t OD_StateMachineInterval_get();
void OD_StateMachineInterval_set(const uint8_t value);

uint8_t OD_StateMachineActivate_get();
void OD_StateMachineActivate_set(const uint8_t value);

uint8_t OD_HVBatteryMode_get();
void OD_HVBatteryMode_set(const uint8_t value);

float OD_EncoderWheelDiameter_get();
void OD_EncoderWheelDiameter_set(const float value);

uint8_t OD_EncoderResetPosition_get();
void OD_EncoderResetPosition_set(const uint8_t value);

uint8_t OD_SetReset_get();
void OD_SetReset_set(const uint8_t value);

uint8_t OD_IMU_number_get();
void OD_IMU_number_set(const uint8_t value);

float OD_IMU1_Temperature_get();
void OD_IMU1_Temperature_set(const float value);

float OD_IMU2_Temperature_get();
void OD_IMU2_Temperature_set(const float value);

float OD_IMU3_Temperature_get();
void OD_IMU3_Temperature_set(const float value);

float OD_IMU_AccelX_get();
void OD_IMU_AccelX_set(const float value);

float OD_IMU_AccelY_get();
void OD_IMU_AccelY_set(const float value);

float OD_IMU_AccelZ_get();
void OD_IMU_AccelZ_set(const float value);

float OD_IMU_GyroX_get();
void OD_IMU_GyroX_set(const float value);

float OD_IMU_GyroY_get();
void OD_IMU_GyroY_set(const float value);

float OD_IMU_GyroZ_get();
void OD_IMU_GyroZ_set(const float value);

float OD_CoolingPressure_get();
void OD_CoolingPressure_set(const float value);

uint16_t OD_FiducialRightCounter_get();
void OD_FiducialRightCounter_set(const uint16_t value);

uint16_t OD_FiducialLeftCounter_get();
void OD_FiducialLeftCounter_set(const uint16_t value);

float OD_Position_get();
void OD_Position_set(const float value);

float OD_Velocity_get();
void OD_Velocity_set(const float value);


/**************************************************************************
* FreeRTOS task that will send out periodically all readable OD entries   *
***************************************************************************/
void sendOdEntriesTask(void *pvParameters);


#endif // CANZERO_SensorF_OD_HPP