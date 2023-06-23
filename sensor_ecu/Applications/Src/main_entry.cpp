/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include <bms44_remote.hpp>
#include <error.hpp>
#include <heartbeat_monitor.hpp>
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
#include "heartbeat_monitor.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	errors::init();
	heartbeat::init();
	info::init();
	ground::init();
	fiducials::init();
	bms44::init();
	brake_remote::init();
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
		heartbeat::update();
		fiducials::update();
		ground::update();
		imu::update();
		brake_remote::update();
		kistler::update();
		bms44::update();
		clu::update();
		cooling::update();
		pdu::update();
		led_status::update();
		errors::update();
		state_maschine::update();
		sdc::update();

		osDelay(pdMS_TO_TICKS(50));
	}
}

#ifdef __cplusplus
}
#endif
