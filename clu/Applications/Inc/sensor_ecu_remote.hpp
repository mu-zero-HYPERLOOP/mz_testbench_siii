/*
 * sensor_ecu.hpp
 *
 *  Created on: May 20, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "State.hpp"

namespace sensor_ecu_remote {

[[nodiscard]] PodState getState();

void init();

void update();

}
