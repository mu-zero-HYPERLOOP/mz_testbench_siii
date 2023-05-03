/*
 * ImuMaster.cpp
 *
 *  Created on: 31.05.2021
 *      Author: Flo Keck
 */

#include "ImuMaster.hpp"
#include "main.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "canzero.hpp"
#include "estdio.hpp"


ImuMaster::ImuMaster() {
	// TODO Auto-generated constructor stub

}

ImuMaster::~ImuMaster() {
	// TODO Auto-generated destructor stub
}

bool ImuMaster::start() {
	// Init all CS
	HAL_GPIO_WritePin(CS_IMU1_GPIO_Port, CS_IMU1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_IMU2_GPIO_Port, CS_IMU2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_IMU3_GPIO_Port, CS_IMU3_Pin, GPIO_PIN_SET);

	// Wait until IMUs should be ready after powerup
	osDelay(pdMS_TO_TICKS(10));

	// Now init all three IMUs
	m_imu1Ok = m_imu1.start();
	m_imu2Ok = m_imu2.start();
	m_imu3Ok = m_imu3.start();

	// Get the number of IMUs
	if(m_imu1Ok) {
		m_numOfImusOnline++;
	}
	if(m_imu2Ok) {
		m_numOfImusOnline++;
	}
	if(m_imu3Ok) {
		m_numOfImusOnline++;
	}
	OD_IMU_number = m_numOfImusOnline;
	printf("IMU-Number = %i\n", m_numOfImusOnline);

	// The internal filters of the IMU need some time to swing in
	osDelay(pdMS_TO_TICKS(100));

	return m_imu1Ok & m_imu2Ok & m_imu3Ok;
}

void ImuMaster::resetValues() {
	m_imu1.resetValues();
	m_imu2.resetValues();
	m_imu3.resetValues();
}

bool ImuMaster::syncRead() {
	if(m_numOfImusOnline == 0) {
		return false;
	}

	// Reset internal variables
	m_accelX = 0;
	m_accelY = 0;
	m_accelZ = 0;
	m_temperature = 0;
	m_gyroX = 0;
	m_gyroY = 0;
	m_gyroZ = 0;

	// Read data from all IMUs that are online
	if(m_imu1Ok) {
		m_imu1.read();

		OD_IMU1_Temperature = m_imu1.getTemperature();

		m_accelX += m_imu1.getAccelX();
		m_accelY += m_imu1.getAccelY();
		m_accelZ += m_imu1.getAccelZ();
		m_temperature += m_imu1.getTemperature();
		m_gyroX += m_imu1.getGyroX();
		m_gyroY += m_imu1.getGyroY();
		m_gyroZ += m_imu1.getGyroZ();
	}

	if(m_imu2Ok) {
		m_imu2.read();

		OD_IMU2_Temperature = m_imu2.getTemperature();

		m_accelX += m_imu2.getAccelX();
		m_accelY += m_imu2.getAccelY();
		m_accelZ += m_imu2.getAccelZ();
		m_temperature += m_imu2.getTemperature();
		m_gyroX += m_imu2.getGyroX();
		m_gyroY += m_imu2.getGyroY();
		m_gyroZ += m_imu2.getGyroZ();
	}

	if(m_imu3Ok) {
		m_imu3.read();

		OD_IMU3_Temperature = m_imu3.getTemperature();

		m_accelX += m_imu3.getAccelX();
		m_accelY += m_imu3.getAccelY();
		m_accelZ += m_imu3.getAccelZ();
		m_temperature += m_imu3.getTemperature();
		m_gyroX += m_imu3.getGyroX();
		m_gyroY += m_imu3.getGyroY();
		m_gyroZ += m_imu3.getGyroZ();
	}

	// Divide data by number of IMUs that are online
	m_accelX /= m_numOfImusOnline;
	m_accelY /= m_numOfImusOnline;
	m_accelZ /= m_numOfImusOnline;
	m_temperature /= m_numOfImusOnline;
	m_gyroX /= m_numOfImusOnline;
	m_gyroY /= m_numOfImusOnline;
	m_gyroZ /= m_numOfImusOnline;

	return true;
}
