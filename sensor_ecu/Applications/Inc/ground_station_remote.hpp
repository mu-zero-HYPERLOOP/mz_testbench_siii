/*
 * ground_station_remote.hpp
 *
 *  Created on: May 27, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include "canzero.hpp"

namespace ground {

using COMMAND = can::signals::SensorF_OD_TelemetryCommands;
using Command = can::signals::SensorF_OD_TelemetryCommands::dataType;

void reset();

[[nodiscard]] Command lastCommand();

void init();

void update();

}
