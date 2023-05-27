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

// Warning: W0_OtherWarning
constexpr uint32_t WARN_OtherWarning_FLAG = 0x1;
void WARN_OtherWarning_set();
void WARN_OtherWarning_clear();
bool WARN_OtherWarning_get();


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


#endif // CANZERO_CLU_EMCY_HPP
