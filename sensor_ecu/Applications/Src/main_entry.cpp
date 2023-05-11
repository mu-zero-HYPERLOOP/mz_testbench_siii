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
#include "EstimatedStateRegistry.hpp"
#include "GroundStationReceiver.hpp"
#include <cmath>

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	//TODO initalize peripherals.
	ImuMaster imuMaster;

	PressureSensor pressureSensor(ADC_MODULE2, 3);

	FiducialSensor fiducialRight = FiducialSensor(
			g_peripherals.m_fiducialRightConfig);
	FiducialSensor fiducialLeft = FiducialSensor(
			g_peripherals.m_fiducialLeftConfig);
	KistlerController kistlerController;

	imuMaster.start();

	while (true) {
		//TODO read sensor data.
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

		// ======= POSITION-ESTIMATION ======
		osDelay(pdMS_TO_TICKS(50));

	}
}

#ifdef __cplusplus
}
#endif
