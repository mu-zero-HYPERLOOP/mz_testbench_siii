/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include "canzero.hpp"
#include "peripheral_config.hpp"
#include "PressureSensor.hpp"
#include "ImuMaster.hpp"
#include "NTCSensor.hpp"
#include "FiducialSensor.hpp"
#include "KistlerController.hpp"
#include <cmath>
#include "pdu_control.hpp"
#include "cooling_controll.hpp"
#include "bms44_receiver.hpp"
#include "NTCSensor.hpp"
#include "mdb_controll.hpp"
#include "kistler_controll.hpp"
#include "imu_controll.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	bms44::init();
	pdu::init();
	mdb::init();
	imu::init();
	kistler::init();

	FiducialSensor fiducialRight = FiducialSensor(
			g_peripherals.m_fiducialRightConfig);
	FiducialSensor fiducialLeft = FiducialSensor(
			g_peripherals.m_fiducialLeftConfig);

	KistlerController kistlerController;

	//MODULE | RANK | R2 | U0 | BETA | R25
	NTCSensor ntc(ADC_MODULE2, 0, 20000, 3.3, 3950, 10000);
	NTCSensor ntc2(ADC_MODULE2, 1, 20000, 3.3, 3977, 10000);

	while (true) {
		unsigned int fiducialLeftCounter = fiducialLeft.getCount();
		OD_FiducialLeftCounter_set((uint16_t)fiducialLeftCounter);

		unsigned int fiducialRightCounter = fiducialRight.getCount();
		OD_FiducialRightCounter_set((uint16_t)fiducialRightCounter);

		float kistlerVel = kistlerController.getVelocity();
		float kistlerPos = kistlerController.getPosition();

		OD_Position_set(kistlerPos);
		OD_Velocity_set(kistlerVel);

		// ======= POSITION-ESTIMATION ======

		imu::update();
		kistler::update();
		bms44::update();
		mdb::update();
		cooling::update();
		pdu::update();

		osDelay(pdMS_TO_TICKS(1000));
	}
}

#ifdef __cplusplus
}
#endif
