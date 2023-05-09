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
#include "SensorValueRegistry.hpp"
#include "KistlerController.hpp"
#include "EstimatedStateRegistry.hpp"
#include "GroundStationReceiver.hpp"
#include <cmath>

#ifdef __cplusplus
extern "C" {
#endif

void main_entry(void *argv) {
	PressureSensor pressureSensor = PressureSensor(
			g_peripherals.m_pressureConfig);
	ImuMaster imuMaster;
	NTCSensor coolingReservoirTemperatur = NTCSensor(
			g_peripherals.m_coolingReservoirTemperaturSensorConfig);
	FiducialSensor fiducialRight = FiducialSensor(
			g_peripherals.m_fiducialRightConfig);
	FiducialSensor fiducialLeft = FiducialSensor(
			g_peripherals.m_fiducialLeftConfig);
	KistlerController kistlerController;

	imuMaster.start();

	while (true) {
		imuMaster.syncRead();

		SensorValueRegistry::getInstance().setGyro(imuMaster.getGyroX(),
				imuMaster.getGyroY(), imuMaster.getGyroZ());

		SensorValueRegistry::getInstance().setAccel(imuMaster.getAccelX(),
				imuMaster.getAccelY(), imuMaster.getAccelZ());

		float coolingReservoirTemp =
				coolingReservoirTemperatur.getTemperaturC();
		SensorValueRegistry::getInstance().setCoolingReservoirTemperatur(
				coolingReservoirTemp);

		float pressure = pressureSensor.getPressure();
		SensorValueRegistry::getInstance().setCoolingPressure(pressure);

		unsigned int fiducialLeftCounter = fiducialLeft.getCount();
		uint32_t fiducialLeftDeltaTime = fiducialLeft.getDeltaTime();
		float fiducialLeftVel = fiducialLeft.estimateVelocityMPS();
		float fiducialLeftPos = fiducialLeft.estimatedPosition();

		SensorValueRegistry::getInstance().setFiducialLeftValues(
				fiducialLeftCounter, fiducialLeftDeltaTime, fiducialLeftVel, fiducialLeftPos);

		unsigned int fiducialRightCounter = fiducialRight.getCount();
		uint32_t fiducialRightDeltaTime = fiducialRight.getDeltaTime();
		float fiducialRightVel = fiducialRight.estimateVelocityMPS();
		float fiducialRightPos = fiducialRight.estimatedPosition();

		SensorValueRegistry::getInstance().setFiducialRightValues(
				fiducialRightCounter, fiducialRightDeltaTime, fiducialRightVel, fiducialRightPos);

		float kistlerVel = kistlerController.getVelocity();
		float kistlerPos = kistlerController.getPosition();
		SensorValueRegistry::getInstance().setKistlerValues(kistlerVel,
				kistlerPos);

		// =========== UPDATE-CAN ===========
		SensorValueRegistry::getInstance().updateCAN();

		// ======= POSITION-ESTIMATION ======
		// compare fiducial left to optical Sensor.
		constexpr float FIDUCIAL_THRESHOLD = 0.1;
		if (	(std::abs(
					SensorValueRegistry::getInstance().getFiducialLeftPosition() - SensorValueRegistry::getInstance().getKistlerPosition())
					> FIDUCIAL_THRESHOLD)
			|| (std::abs(
					SensorValueRegistry::getInstance().getFiducialRightPosition() - SensorValueRegistry::getInstance().getKistlerPosition())
					> FIDUCIAL_THRESHOLD)) {
			//ERR_fiducialHighOffset_set();
			//printf("fiducial position estimation derived from kistler\n");
		}

		EstimatedStateRegistry::getInstance().setPosition(kistlerPos);

		printf("Fiducial Counter : %d\n", SensorValueRegistry::getInstance().getFiducialLeftCount());

		osDelay(pdMS_TO_TICKS(1000));

	}
}

#ifdef __cplusplus
}
#endif
