/*
 * sensor_ecu_remote.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "State.hpp"

namespace sensor_ecu {

void init();

void update();

PodState getState();


}
