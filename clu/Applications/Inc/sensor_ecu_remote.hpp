/*
 * sensor_ecu.hpp
 *
 *  Created on: May 20, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "State.hpp"

namespace sensor_ecu_remote {

enum ActionRequest {
	MDB_START,
	MDB_STOP,
};

[[nodiscard]] PodState getState();

[[nodiscard]] ActionRequest getLastActionRequest();

void init();

void update();

}
