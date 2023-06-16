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
	MDB_STATE_INIT = 0,
	MDB_STATE_IDLE = 1,
	MDB_STATE_PRECHARGE = 2,
	MDB_STATE_READY = 3,
	MDB_STATE_LEVI_START = 4,
	MDB_STATE_LEVI_RUN = 5,
	MDB_STATE_LEVI_END = 6,
	MDB_STATE_LEVI_UNSTABLE = 7,
	MDB_STATE_ERROR = 10,
	MDB_ERROR_OVERCURRENT = 11,
	MDB_ERROR_OVERVOLT = 12,
	MDB_ERROR_OVERTEMP = 13,
	MDB_ERROR_AIRGAP = 14,
	MDB_ERROR_AIRGAP_SEN = 15,
	MDB_ERROR_CAN = 16,
	MDB_ERROR_CURRE_CALIB = 17,
	MDB_ERROR_CHIPTEMP = 18,
	MDB_ERROR_CALCTIME = 19,

	MDB_STATE_COUNT = MDB_ERROR_CALCTIME + 1
};

enum PublicState : uint8_t {
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

enum Command : uint8_t {
	MDB_COMMAND_NONE = 0,
	MDB_COMMAND_PREPARE = 1, //ignore target airgap.
	MDB_COMMAND_START = 2, //with target airgap.
	MDB_COMMAND_STOP = 3, //ignore target airgap.
	MDB_COMMAND_SET_AIRGAP = 4, //with target airgap.
};

[[nodiscard]] PublicState getState();

void setTargetAirGap(float airGap);

void setCommand(Command command);

float getAirGap(uint8_t mdbId);

float getTemperature(uint8_t mdbId);

void init();

void update();

}
