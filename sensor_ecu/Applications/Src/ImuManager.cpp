/*
 * ImuManager.cpp
 *
 *  Created on: 31.05.2021
 *      Author: Flo Keck
 */

#include "ImuManager.hpp"
#include "main.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "cz_interface.hpp"


ImuManager::ImuManager() {
	// TODO Auto-generated constructor stub

}

ImuManager::~ImuManager() {
	// TODO Auto-generated destructor stub
}

bool ImuManager::start() {
	// Init all CS
	HAL_GPIO_WritePin(CS_IMU1_GPIO_Port, CS_IMU1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_IMU2_GPIO_Port, CS_IMU2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS_IMU3_GPIO_Port, CS_IMU3_Pin, GPIO_PIN_SET);

	// Wait until IMUs should be ready after powerup
	osDelay(pdMS_TO_TICKS(10));

	// Now init all three IMUs
	m_imu1Ok = imu1.start();
	m_imu2Ok = imu2.start();
	m_imu3Ok = imu3.start();

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

	// The internal filters of the IMU need some time to swing in
	osDelay(pdMS_TO_TICKS(100));

	return m_imu1Ok & m_imu2Ok & m_imu3Ok;
}

void ImuManager::resetValues() {
	imu1.resetValues();
	imu2.resetValues();
	imu3.resetValues();
}

bool ImuManager::read() {
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
		imu1.read();

		OD_IMU1_Temperature = imu1.getTemperature();

		m_accelX += imu1.getAccelX();
		m_accelY += imu1.getAccelY();
		m_accelZ += imu1.getAccelZ();
		m_temperature += imu1.getTemperature();
		m_gyroX += imu1.getGyroX();
		m_gyroY += imu1.getGyroY();
		m_gyroZ += imu1.getGyroZ();
	}

	if(m_imu2Ok) {
		imu2.read();

		OD_IMU2_Temperature = imu2.getTemperature();

		m_accelX += imu2.getAccelX();
		m_accelY += imu2.getAccelY();
		m_accelZ += imu2.getAccelZ();
		m_temperature += imu2.getTemperature();
		m_gyroX += imu2.getGyroX();
		m_gyroY += imu2.getGyroY();
		m_gyroZ += imu2.getGyroZ();
	}

	if(m_imu3Ok) {
		imu3.read();

		OD_IMU3_Temperature = imu3.getTemperature();

		m_accelX += imu3.getAccelX();
		m_accelY += imu3.getAccelY();
		m_accelZ += imu3.getAccelZ();
		m_temperature += imu3.getTemperature();
		m_gyroX += imu3.getGyroX();
		m_gyroY += imu3.getGyroY();
		m_gyroZ += imu3.getGyroZ();
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
