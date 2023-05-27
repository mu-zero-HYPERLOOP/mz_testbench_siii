/*
 * mdb_remote.hpp
 *
 *  Created on: May 20, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include <cinttypes>

namespace mdb {

static constexpr uint8_t MDB_COUNT = 6;

enum State : uint8_t{
	MDB_PRECHARGE = 0,
	MDB_OFF = 1,
	MDB_STATE_COUNT = 2,
	MDB_INCONSISTANT = MDB_STATE_COUNT,
};

[[nodiscard]] State getState();

void setTargetAirGap(uint8_t mdbId, float airGap);

float getAirGap(uint8_t mdbId);

float getTemperature(uint8_t mdbId);

void init();

void update();

}
