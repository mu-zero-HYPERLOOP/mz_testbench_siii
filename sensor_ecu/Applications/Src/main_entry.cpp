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

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	//bms44::init();

	//PressureSensor pressureSensor(ADC_MODULE2, 3);
	pdu::init();
	bool toggle = true;
	pdu::enableChannel(pdu::LP_CHANNEL3);
	//TODO initalize peripherals.
	/*
	ImuMaster imuMaster;


	FiducialSensor fiducialRight = FiducialSensor(
			g_peripherals.m_fiducialRightConfig);
	FiducialSensor fiducialLeft = FiducialSensor(
			g_peripherals.m_fiducialLeftConfig);
	KistlerController kistlerController;

	imuMaster.start();
	*/


	while (true) {
		pdu::enableChannel(pdu::LP_CHANNEL3);
		osDelay(pdMS_TO_TICKS(1000));
		pdu::update();
		osDelay(pdMS_TO_TICKS(1000));
		pdu::disableChannel(pdu::LP_CHANNEL3);
		osDelay(pdMS_TO_TICKS(1000));
		pdu::update();
		osDelay(pdMS_TO_TICKS(1000));
		//bms44::update();
		//TODO read sensor data.
		/*
		imuMaster.syncRead();
		OD_IMU_AccelX_set(imuMaster.getAccelX());
		OD_IMU_AccelY_set(imuMaster.getAccelY());
		OD_IMU_AccelZ_set(imuMaster.getAccelZ());

		OD_IMU_GyroX_set(imuMaster.getGyroX());
		OD_IMU_GyroY_set(imuMaster.getGyroY());
		OD_IMU_GyroZ_set(imuMaster.getGyroZ());

		OD_CoolingPressure_set(pressureSensor.get());

		unsigned int fiducialLeftCounter = fiducialLeft.getCount();
		OD_FiducialLeftCounter_set((uint16_t)fiducialLeftCounter);

		unsigned int fiducialRightCounter = fiducialRight.getCount();
		OD_FiducialRightCounter_set((uint16_t)fiducialRightCounter);

		float kistlerVel = kistlerController.getVelocity();
		float kistlerPos = kistlerController.getPosition();

		OD_Position_set(kistlerPos);
		OD_Velocity_set(kistlerVel);


		cooling::update();
		*/
		// ======= POSITION-ESTIMATION ======

	}
}

#ifdef __cplusplus
}
#endif
