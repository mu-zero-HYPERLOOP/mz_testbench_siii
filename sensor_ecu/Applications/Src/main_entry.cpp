/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include <bms44_remote.hpp>
#include <imu.hpp>
#include <kistler_remote.hpp>
#include <mdb_remote.hpp>
#include <pdu_remote.hpp>
#include <proc_info.hpp>
#include "canzero.hpp"
#include "ImuMaster.hpp"
#include "NTCSensor.hpp"
#include "brake_ecu_remote.hpp"
#include <cmath>
#include "cooling_controll.hpp"
#include "NTCSensor.hpp"
//#include "fiducials.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	info::init();
	//fiducials::init();
	//bms44::init();
	brake::init();
	cooling::init();
	pdu::init(false);
	mdb::init();
	imu::init();
	kistler::init();

	while (true) {
		info::update();
		//fiducials::update();
		imu::update();
		brake::update();
		kistler::update();
		//bms44::update();
		mdb::update();
		cooling::update();
		pdu::update();

		osDelay(pdMS_TO_TICKS(100));
	}
}

#ifdef __cplusplus
}
#endif
