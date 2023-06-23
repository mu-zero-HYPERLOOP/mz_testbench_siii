/*
 * GlobalState.hpp
 *
 *  Created on: Jun 18, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include "cz_interface.hpp"
#include <cinttypes>

namespace state {

using STATE = can::signals::SensorF_TX_PodState;
using PodState = uint8_t;

PodState get();

void receiveCAN();

}
