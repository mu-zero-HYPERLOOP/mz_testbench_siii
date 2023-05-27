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
	MDB_PRECHARGE = 0,
	MDB_OFF = 1,
	MDB_READY = 2,
	MDB_STABLE_LEV = 3,
	MDB_GROUNDED = 4,
	MDB_STATE_COUNT = MDB_GROUNDED + 1,
	MDB_INCONSISTANT = MDB_STATE_COUNT,
};

enum Action : uint8_t {
	MDB_START,
	MDB_STOP
};

[[nodiscard]] State getState();

[[nodiscard]] bool requiresCooling();

void requestAction(Action action);

void init();

void update();

}
