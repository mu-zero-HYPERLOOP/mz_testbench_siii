/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This header file was generated from 'pod2023_gen.dbc'.
 * It contains the errors and warnings for the node 'PDU'.
 *
 * Florian Keck
 * florian.keck@mu-zero.de
 * Copyright 2023, mu-zero HYPERLOOP e.V.
 */
#ifndef CANZERO_PDU_EMCY_HPP
#define CANZERO_PDU_EMCY_HPP

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

// Warning: W1_batterVoltageLow
constexpr uint32_t WARN_batterVoltageLow_FLAG = 0x2;
void WARN_batterVoltageLow_set();
void WARN_batterVoltageLow_clear();
bool WARN_batterVoltageLow_get();

// Warning: W2_batterTempHigh
constexpr uint32_t WARN_batterTempHigh_FLAG = 0x4;
void WARN_batterTempHigh_set();
void WARN_batterTempHigh_clear();
bool WARN_batterTempHigh_get();


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

// Error: E1_batterVoltageCritical
constexpr uint32_t ERR_batterVoltageCritical_FLAG = 0x2000;
void ERR_batterVoltageCritical_set();
void ERR_batterVoltageCritical_clear();
bool ERR_batterVoltageCritical_get();

// Error: E2_batteryOvercurrent
constexpr uint32_t ERR_batteryOvercurrent_FLAG = 0x4000;
void ERR_batteryOvercurrent_set();
void ERR_batteryOvercurrent_clear();
bool ERR_batteryOvercurrent_get();

// Error: E3_batterTempCritical
constexpr uint32_t ERR_batterTempCritical_FLAG = 0x8000;
void ERR_batterTempCritical_set();
void ERR_batterTempCritical_clear();
bool ERR_batterTempCritical_get();

// Error: E4_watchdogStateMachine
constexpr uint32_t ERR_watchdogStateMachine_FLAG = 0x10000;
void ERR_watchdogStateMachine_set();
void ERR_watchdogStateMachine_clear();
bool ERR_watchdogStateMachine_get();


#endif // CANZERO_PDU_EMCY_HPP
