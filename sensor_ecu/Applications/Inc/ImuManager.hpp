/*
 * ImuManager.hpp
 *
 * A small class that manages the three IMUs on the sensor shield.
 * The IMU Manager averages the values of the three IMUs and also does the transformation from IMU to Pod coordinate system.
 * It also works, if one or two IMU fails, then only the data from the working IMUs will be used.
 *
 * The X and Z axis are swapped and Y and Z are inverted to translate form IMU to pod coordinate system.
 * Its different to SensorF!
 *
 *  Created on: 24.05.2021
 *      Author: Max Moebius
 */

#ifndef INCLUDE_IMUMANAGER_HPP_
#define INCLUDE_IMUMANAGER_HPP_

#include "ICM20602.hpp"

extern SPI_HandleTypeDef hspi2;

class ImuManager {
private:
	ICM20602::config_t imuConfig = {
		.accelDlpf = 		ICM20602::ACCEL_DLPF_44_8_HZ,
		.accelFs = 			ICM20602::ACCEL_FS_4G,
		.gyroDlpf = 		ICM20602::GYRO_DLPF_41_HZ,
		.gyroDps = 			ICM20602::GYRO_RANGE_500_DPS,
		.sampleRateDiv = 	1,	// 1kHz / (1 + 1) = 500Hz -> faster then we read it -> fine
	};

	ICM20602 imu1{imuConfig, &hspi2, CS_IMU1_GPIO_Port, CS_IMU1_Pin};
	ICM20602 imu2{imuConfig, &hspi2, CS_IMU2_GPIO_Port, CS_IMU2_Pin};
	ICM20602 imu3{imuConfig, &hspi2, CS_IMU3_GPIO_Port, CS_IMU3_Pin};

	/**
	 * Variables to read the data
	 */
	float m_accelX = 0.0f;
	float m_accelY = 0.0f;
	float m_accelZ = 0.0f;
	float m_temperature = 0.0f;
	float m_gyroX = 0.0f;
	float m_gyroY = 0.0f;
	float m_gyroZ = 0.0f;

	bool m_imu1Ok = false;
	bool m_imu2Ok = false;
	bool m_imu3Ok = false;

	int m_numOfImusOnline = 0;

public:
	ImuManager();
	virtual ~ImuManager();

	/**
	 * start all three IMUs.
	 * @return True if all IMUs were inited successful
	 */
	bool start();

	/**
	 * Zero all Gyro and Accel values! This can be only called when the IMU is still!
	 */
	void resetValues();

	/**
	 * Read data from the IMUs.
	 * @return True if there was no error.
	 */
	bool read();

	/**
	 * Get acceleration in X direction of the pod in m/s^2.
	 * @return Acceleration in X direction of the pod in m/s^2.
	 */
	inline float getAccelX() { return m_accelZ; }

	/**
	 * Get acceleration in Y direction of the pod in m/s^2.
	 * @return Acceleration in Y direction of the pod in m/s^2.
	 */
	inline float getAccelY() { return -m_accelY; }

	/**
	 * Get acceleration in Z direction of the pod in m/s^2.
	 * @return Acceleration in Z direction of the pod in m/s^2.
	 */
	inline float getAccelZ() { return -m_accelX; }

	/**
	 * Get the average internal temperature of the IMU in °C.
	 * @return IMU temperature in °C.
	 */
	inline float getTemperature() { return m_temperature; }

	/**
	 * Get rotation around X axis of the pod in °/s.
	 * @return Rotation around X axis in °/s.
	 */
	inline float getGyroX() { return m_gyroX; }

	/**
	 * Get rotation around Y axis of the pod in °/s.
	 * @return Rotation around Y axis in °/s.
	 */
	inline float getGyroY() { return m_gyroZ; }

	/**
	 * Get rotation around Z axis of the pod in °/s.
	 * @return Rotation around Z axis in °/s.
	 */
	inline float getGyroZ() { return -m_gyroY; }
};

#endif /* INCLUDE_IMUMANAGER_HPP_ */
