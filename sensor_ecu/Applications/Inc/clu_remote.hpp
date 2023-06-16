/*
 * clu_remote.hpp
 *
 *  Created on: May 21, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include <cinttypes>

namespace clu {

enum State : uint8_t{
	MDB_OFF = 0,
	MDB_IDLE = 1,
	MDB_PRECHARGE = 2,
	MDB_READY = 3,
	MDB_LEVI_START = 4,
	MDB_LEVI = 5,
	MDB_ERROR = 6,
	MDB_INCONSISTANT = 7,
	MDB_PUBLIC_STATE_COUNT = MDB_INCONSISTANT + 1,
};

[[nodiscard]] State getState();

[[nodiscard]] bool requiresCooling();

void init();

void update();

}
