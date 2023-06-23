/*
 * state_maschine.hpp
 *
 *  Created on: May 11, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "canzero.hpp"


namespace state_maschine {

using STATE = can::signals::SensorF_TX_PodState;

using PodState = uint8_t;

void setState(PodState state);

PodState getState();

void init();

void update();

}
