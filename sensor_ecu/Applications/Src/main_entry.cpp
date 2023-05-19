/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include <bms44_remote.hpp>
#include <imu.hpp>
#include <mdb_remote.hpp>
#include <pdu_remote.hpp>
#include <proc_info.hpp>
#include "canzero.hpp"
#include "ImuMaster.hpp"
#include "NTCSensor.hpp"
#include "FiducialSensor.hpp"
#include <cmath>
#include "cooling_controll.hpp"
#include "NTCSensor.hpp"
#include "kistler_controll.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	info::init();
	//bms44::init();
	cooling::init();
	pdu::init(false);
	mdb::init();
	imu::init();
	kistler::init();

	while (true) {
		info::update();
		imu::update();
		kistler::update();
		//bms44::update();
		mdb::update();
		cooling::update();
		pdu::update();

		osDelay(pdMS_TO_TICKS(500));
	}
}

#ifdef __cplusplus
}
#endif
