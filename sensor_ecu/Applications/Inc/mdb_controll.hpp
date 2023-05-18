/*
 * mdb_controll.hpp
 *
 *  Created on: May 18, 2023
 *      Author: OfficeLaptop
 */

#pragma once

namespace mdb {

enum MdbState {
	MDB_STATE_PRECHARGE = 0,
	MDB_STATE_LEVITATION = 1,
	MDB_STATE_GROUNDED = 2,
	MDB_STATE_SAVE_TO_APPROCH = 3,
	MDB_STATE_PRECHARGE_DONE = 4,
	MDB_STATE_INCONSISTANT = 255,
};

void init();

void update();

}
