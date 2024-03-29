/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This header file was generated from 'pod2023_gen.dbc'.
 * It contains the object dictionary for the node 'Track'.
 *
 * Florian Keck
 * florian.keck@mu-zero.de
 * Copyright 2023, mu-zero HYPERLOOP e.V.
 */
#ifndef CANZERO_Track_OD_HPP
#define CANZERO_Track_OD_HPP

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
extern volatile uint8_t  OD_PistonStatus;
extern volatile float    OD_PropulsionDistance;
extern volatile float    OD_PressureReservoir;
extern volatile float    OD_PressurePush;
extern volatile float    OD_PressureRetract;

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
extern osMutexId_t mutex_OD_PistonStatus;
extern osMutexId_t mutex_OD_PropulsionDistance;
extern osMutexId_t mutex_OD_PressureReservoir;
extern osMutexId_t mutex_OD_PressurePush;
extern osMutexId_t mutex_OD_PressureRetract;


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

uint8_t OD_PistonStatus_get();
void OD_PistonStatus_set(const uint8_t value);

float OD_PropulsionDistance_get();
void OD_PropulsionDistance_set(const float value);

float OD_PressureReservoir_get();
void OD_PressureReservoir_set(const float value);

float OD_PressurePush_get();
void OD_PressurePush_set(const float value);

float OD_PressureRetract_get();
void OD_PressureRetract_set(const float value);


/**************************************************************************
* FreeRTOS task that will send out periodically all readable OD entries   *
***************************************************************************/
void sendOdEntriesTask(void *pvParameters);


#endif // CANZERO_Track_OD_HPP