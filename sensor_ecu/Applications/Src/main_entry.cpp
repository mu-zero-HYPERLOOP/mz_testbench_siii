/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include <bms44_remote.hpp>
#include "ground_station_remote.hpp"
#include <imu.hpp>
#include <kistler_remote.hpp>
#include "clu_remote.hpp"
#include <pdu_remote.hpp>
#include <proc_info.hpp>
#include "canzero.hpp"
#include "ImuMaster.hpp"
#include "brake_ecu_remote.hpp"
#include <cmath>
#include "cooling_controll.hpp"
#include "fiducials.hpp"
#include "led_status.hpp"
#include "sdc.hpp"
#include "state_maschine.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	info::init();
	ground::init();
	fiducials::init();
	bms44::init();
	brake::init();
	cooling::init();
	pdu::init(false);
	clu::init();
	imu::init();
	kistler::init();
	led_status::init();
	sdc::init();
	state_maschine::init();

	while (true) {
		info::update();
		ground::update();
		fiducials::update();
		imu::update();
		brake::update();
		kistler::update();
		bms44::update();
		clu::update();
		cooling::update();
		pdu::update();
		led_status::update();
		sdc::update();
		state_maschine::update();

		osDelay(pdMS_TO_TICKS(50));
	}
}

#ifdef __cplusplus
}
#endif
