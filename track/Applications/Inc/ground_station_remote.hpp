/*
 * ground_station_remote.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "State.hpp"

namespace ground_station {

void init();

void update();

PodState getState();

}
