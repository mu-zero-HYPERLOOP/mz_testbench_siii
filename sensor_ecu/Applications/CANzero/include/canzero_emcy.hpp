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

// Error: E1_CPUOverTemp
constexpr uint32_t ERR_CPUOverTemp_FLAG = 0x2000;
void ERR_CPUOverTemp_set();
void ERR_CPUOverTemp_clear();
bool ERR_CPUOverTemp_get();

// Error: E2_OverVolt
constexpr uint32_t ERR_OverVolt_FLAG = 0x4000;
void ERR_OverVolt_set();
void ERR_OverVolt_clear();
bool ERR_OverVolt_get();

// Error: E3_UnderVolt
constexpr uint32_t ERR_UnderVolt_FLAG = 0x8000;
void ERR_UnderVolt_set();
void ERR_UnderVolt_clear();
bool ERR_UnderVolt_get();

// Error: E4_InvalidPosition
constexpr uint32_t ERR_InvalidPosition_FLAG = 0x10000;
void ERR_InvalidPosition_set();
void ERR_InvalidPosition_clear();
bool ERR_InvalidPosition_get();

// Error: E5_ReservoirOverTemp
constexpr uint32_t ERR_ReservoirOverTemp_FLAG = 0x20000;
void ERR_ReservoirOverTemp_set();
void ERR_ReservoirOverTemp_clear();
bool ERR_ReservoirOverTemp_get();

// Error: E6_CLUHeartbeatMiss
constexpr uint32_t ERR_CLUHeartbeatMiss_FLAG = 0x40000;
void ERR_CLUHeartbeatMiss_set();
void ERR_CLUHeartbeatMiss_clear();
bool ERR_CLUHeartbeatMiss_get();

// Error: E7_BECUHeartbeatMiss
constexpr uint32_t ERR_BECUHeartbeatMiss_FLAG = 0x80000;
void ERR_BECUHeartbeatMiss_set();
void ERR_BECUHeartbeatMiss_clear();
bool ERR_BECUHeartbeatMiss_get();

// Error: E8_PDUHeartbeatMiss
constexpr uint32_t ERR_PDUHeartbeatMiss_FLAG = 0x100000;
void ERR_PDUHeartbeatMiss_set();
void ERR_PDUHeartbeatMiss_clear();
bool ERR_PDUHeartbeatMiss_get();

// Error: E9_TelemetryHeartbeatMiss
constexpr uint32_t ERR_TelemetryHeartbeatMiss_FLAG = 0x200000;
void ERR_TelemetryHeartbeatMiss_set();
void ERR_TelemetryHeartbeatMiss_clear();
bool ERR_TelemetryHeartbeatMiss_get();

// Error: E10_TitanOverTemp
constexpr uint32_t ERR_TitanOverTemp_FLAG = 0x400000;
void ERR_TitanOverTemp_set();
void ERR_TitanOverTemp_clear();
bool ERR_TitanOverTemp_get();

// Error: E11_HyperionOverTemp
constexpr uint32_t ERR_HyperionOverTemp_FLAG = 0x800000;
void ERR_HyperionOverTemp_set();
void ERR_HyperionOverTemp_clear();
bool ERR_HyperionOverTemp_get();

// Error: E12_TitanLowHp
constexpr uint32_t ERR_TitanLowHp_FLAG = 0x1000000;
void ERR_TitanLowHp_set();
void ERR_TitanLowHp_clear();
bool ERR_TitanLowHp_get();

// Error: E13_HyperionLowHp
constexpr uint32_t ERR_HyperionLowHp_FLAG = 0x2000000;
void ERR_HyperionLowHp_set();
void ERR_HyperionLowHp_clear();
bool ERR_HyperionLowHp_get();

// Error: E14_TitanLowCap
constexpr uint32_t ERR_TitanLowCap_FLAG = 0x4000000;
void ERR_TitanLowCap_set();
void ERR_TitanLowCap_clear();
bool ERR_TitanLowCap_get();

// Error: E15_HyperionLowCap
constexpr uint32_t ERR_HyperionLowCap_FLAG = 0x8000000;
void ERR_HyperionLowCap_set();
void ERR_HyperionLowCap_clear();
bool ERR_HyperionLowCap_get();

// Error: E16_EboxOverTemp
constexpr uint32_t ERR_EboxOverTemp_FLAG = 0x10000000;
void ERR_EboxOverTemp_set();
void ERR_EboxOverTemp_clear();
bool ERR_EboxOverTemp_get();


#endif // CANZERO_SensorF_EMCY_HPP
