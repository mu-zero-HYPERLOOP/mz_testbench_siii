/* DO NOT MODIFY. THIS FILE WAS GENERATED AUTOMATICALLY BY CZ2CPP V1.7.7.
 *
 * This header file was generated from 'pod2023_gen.dbc'.
 * It contains the errors and warnings for the node 'CLU'.
 *
 * Florian Keck
 * florian.keck@mu-zero.de
 * Copyright 2023, mu-zero HYPERLOOP e.V.
 */
#ifndef CANZERO_CLU_EMCY_HPP
#define CANZERO_CLU_EMCY_HPP

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

// Warning: W0_highPressureActingChamber
constexpr uint32_t WARN_highPressureActingChamber_FLAG = 0x1;
void WARN_highPressureActingChamber_set();
void WARN_highPressureActingChamber_clear();
bool WARN_highPressureActingChamber_get();

// Warning: W1_highPressureRetractingChamber
constexpr uint32_t WARN_highPressureRetractingChamber_FLAG = 0x2;
void WARN_highPressureRetractingChamber_set();
void WARN_highPressureRetractingChamber_clear();
bool WARN_highPressureRetractingChamber_get();

// Warning: W2_enableWithAnError
constexpr uint32_t WARN_enableWithAnError_FLAG = 0x4;
void WARN_enableWithAnError_set();
void WARN_enableWithAnError_clear();
bool WARN_enableWithAnError_get();

// Warning: W3_externalError
constexpr uint32_t WARN_externalError_FLAG = 0x8;
void WARN_externalError_set();
void WARN_externalError_clear();
bool WARN_externalError_get();


/**************************************************************************
* Functions to get, set and reset CANzero errors                          *
***************************************************************************/
// Use this flag to test for any error
constexpr uint32_t ERR_ANY_FLAG = 0xFFFFF000;

// Reset all errors
void ERR_ALL_clear();

// Error: E0_pressureTooHigh
constexpr uint32_t ERR_pressureTooHigh_FLAG = 0x1000;
void ERR_pressureTooHigh_set();
void ERR_pressureTooHigh_clear();
bool ERR_pressureTooHigh_get();

// Error: E1_pressureTooLow
constexpr uint32_t ERR_pressureTooLow_FLAG = 0x2000;
void ERR_pressureTooLow_set();
void ERR_pressureTooLow_clear();
bool ERR_pressureTooLow_get();

// Error: E2_commWatchdogTimeout
constexpr uint32_t ERR_commWatchdogTimeout_FLAG = 0x4000;
void ERR_commWatchdogTimeout_set();
void ERR_commWatchdogTimeout_clear();
bool ERR_commWatchdogTimeout_get();

// Error: E3_retractUnsuccesful_errorFlag
constexpr uint32_t ERR_retractUnsuccesful_errorFlag_FLAG = 0x8000;
void ERR_retractUnsuccesful_errorFlag_set();
void ERR_retractUnsuccesful_errorFlag_clear();
bool ERR_retractUnsuccesful_errorFlag_get();

// Error: E4_retractUnsuccesful_notEnabled
constexpr uint32_t ERR_retractUnsuccesful_notEnabled_FLAG = 0x10000;
void ERR_retractUnsuccesful_notEnabled_set();
void ERR_retractUnsuccesful_notEnabled_clear();
bool ERR_retractUnsuccesful_notEnabled_get();

// Error: E5_retractUnsuccesful_openSDC
constexpr uint32_t ERR_retractUnsuccesful_openSDC_FLAG = 0x20000;
void ERR_retractUnsuccesful_openSDC_set();
void ERR_retractUnsuccesful_openSDC_clear();
bool ERR_retractUnsuccesful_openSDC_get();


#endif // CANZERO_CLU_EMCY_HPP
