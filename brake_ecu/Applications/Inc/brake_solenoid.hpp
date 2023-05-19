/*
 * brake_solenoid.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include <cinttypes>

namespace brake_solenoid {

enum BrakeStatus : uint8_t {
	Disengage = 0,
	Engage = 1
};

void init();

void engage();

void disengage();

}
