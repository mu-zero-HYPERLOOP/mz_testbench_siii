/*
 * ground_station_remote.hpp
 *
 *  Created on: May 27, 2023
 *      Author: OfficeLaptop
 */

#pragma once

namespace ground {

enum Command {
	COMMAND_NONE,
	COMMAND_PREPARE,
	COMMAND_LAUNCH,
	COMMAND_PUSHABLE,
};

void reset();

[[nodiscard]] Command lastCommand();

void init();

void update();

}
