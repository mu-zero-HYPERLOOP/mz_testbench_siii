/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This header file was generated from 'pod2023_gen.dbc'.
 * It contains the errors and warnings for the node 'SensorF'.
 *
 * Florian Keck
 * florian.keck@mu-zero.de
 * Copyright 2023, mu-zero HYPERLOOP e.V.
 */
#ifndef CANZERO_SensorF_EMCY_HPP
#define CANZERO_SensorF_EMCY_HPP

#pragma once

#include "cz_interface.hpp"
#include "dbc_parser.hpp"


/**************************************************************************
* Functions to get, set and reset CANzero warnings                        *
***************************************************************************/
// Use this flag to test for any warning
constexpr uint32_t WARN_ANY_FLAG = 0xFFF;

// Reset all warnings
void WARN_ALL_clear();

// Warning: W0_OtherWarning
constexpr uint32_t WARN_OtherWarning_FLAG = 0x1;
void WARN_OtherWarning_set();
void WARN_OtherWarning_clear();
bool WARN_OtherWarning_get();

// Warning: W1_StateMTransitionW
constexpr uint32_t WARN_StateMTransitionW_FLAG = 0x2;
void WARN_StateMTransitionW_set();
void WARN_StateMTransitionW_clear();
bool WARN_StateMTransitionW_get();

// Warning: W2_encoderOORWarning
constexpr uint32_t WARN_encoderOORWarning_FLAG = 0x4;
void WARN_encoderOORWarning_set();
void WARN_encoderOORWarning_clear();
bool WARN_encoderOORWarning_get();


/**************************************************************************
* Functions to get, set and reset CANzero errors                          *
***************************************************************************/
// Use this flag to test for any error
constexpr uint32_t ERR_ANY_FLAG = 0xFFFFF000;

// Reset all errors
void ERR_ALL_clear();

// Error: E0_OtherError
constexpr uint32_t ERR_OtherError_FLAG = 0x1000;
void ERR_OtherError_set();
void ERR_OtherError_clear();
bool ERR_OtherError_get();

// Error: E1_StateMTransitionE
constexpr uint32_t ERR_StateMTransitionE_FLAG = 0x2000;
void ERR_StateMTransitionE_set();
void ERR_StateMTransitionE_clear();
bool ERR_StateMTransitionE_get();

// Error: E2_BrakeFTimeout
constexpr uint32_t ERR_BrakeFTimeout_FLAG = 0x4000;
void ERR_BrakeFTimeout_set();
void ERR_BrakeFTimeout_clear();
bool ERR_BrakeFTimeout_get();

// Error: E3_BrakeRTimeout
constexpr uint32_t ERR_BrakeRTimeout_FLAG = 0x8000;
void ERR_BrakeRTimeout_set();
void ERR_BrakeRTimeout_clear();
bool ERR_BrakeRTimeout_get();

// Error: E4_PDUTimeout
constexpr uint32_t ERR_PDUTimeout_FLAG = 0x10000;
void ERR_PDUTimeout_set();
void ERR_PDUTimeout_clear();
bool ERR_PDUTimeout_get();

// Error: E5_HVCUTimeout
constexpr uint32_t ERR_HVCUTimeout_FLAG = 0x20000;
void ERR_HVCUTimeout_set();
void ERR_HVCUTimeout_clear();
bool ERR_HVCUTimeout_get();

// Error: E6_SensorRTimeout
constexpr uint32_t ERR_SensorRTimeout_FLAG = 0x40000;
void ERR_SensorRTimeout_set();
void ERR_SensorRTimeout_clear();
bool ERR_SensorRTimeout_get();

// Error: E7_TelemetryTimeout
constexpr uint32_t ERR_TelemetryTimeout_FLAG = 0x80000;
void ERR_TelemetryTimeout_set();
void ERR_TelemetryTimeout_clear();
bool ERR_TelemetryTimeout_get();

// Error: E8_NodeErrorFlag
constexpr uint32_t ERR_NodeErrorFlag_FLAG = 0x100000;
void ERR_NodeErrorFlag_set();
void ERR_NodeErrorFlag_clear();
bool ERR_NodeErrorFlag_get();

// Error: E9_SWError
constexpr uint32_t ERR_SWError_FLAG = 0x200000;
void ERR_SWError_set();
void ERR_SWError_clear();
bool ERR_SWError_get();

// Error: E10_TelemEmergency
constexpr uint32_t ERR_TelemEmergency_FLAG = 0x400000;
void ERR_TelemEmergency_set();
void ERR_TelemEmergency_clear();
bool ERR_TelemEmergency_get();

// Error: E12_encoderError
constexpr uint32_t ERR_encoderError_FLAG = 0x1000000;
void ERR_encoderError_set();
void ERR_encoderError_clear();
bool ERR_encoderError_get();

// Error: E13_encoderSpeedError
constexpr uint32_t ERR_encoderSpeedError_FLAG = 0x2000000;
void ERR_encoderSpeedError_set();
void ERR_encoderSpeedError_clear();
bool ERR_encoderSpeedError_get();

// Error: E14_fiducialHighOffset
constexpr uint32_t ERR_fiducialHighOffset_FLAG = 0x4000000;
void ERR_fiducialHighOffset_set();
void ERR_fiducialHighOffset_clear();
bool ERR_fiducialHighOffset_get();


#endif // CANZERO_SensorF_EMCY_HPP
