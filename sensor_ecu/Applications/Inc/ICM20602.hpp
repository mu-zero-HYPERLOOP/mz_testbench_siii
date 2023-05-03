/*
 * ICM20602.hpp
 *
 *  Created on: May 30, 2021
 *      Author: Flo Keck
 *
 *  SPI driver for TDK InvenSense ICM20602 IMU
 *
 *  Reimplemented https://github.com/mreutman/icm20602
 *
 *  CubeMX config:
 *  	ICM20602 supports up to 10 Mbit/s -> I chose 5.25 Mbit/s
 *  	Bit order: MSB first
 *  	"Data is latched on the rising edge of SPC" -> CPOL = 0
 *  	"Data should be transitioned on the falling edge of SPC" -> CPHA = 0
 *
 */

#ifndef INCLUDE_ICM20602_HPP_
#define INCLUDE_ICM20602_HPP_

#include "main.h"

class ICM20602 {
public:
	/**
	 *
	 */
	typedef enum {
		ACCEL_DLPF_218_1_HZ = 0,  /**< ACCEL_DLPF_218_1_HZ */
		ACCEL_DLPF_99_HZ = 2,     /**< ACCEL_DLPF_99_HZ */
		ACCEL_DLPF_44_8_HZ = 3,   /**< ACCEL_DLPF_44_8_HZ */
		ACCEL_DLPF_21_2_HZ = 4,   /**< ACCEL_DLPF_21_2_HZ */
		ACCEL_DLPF_10_2_HZ = 5,   /**< ACCEL_DLPF_10_2_HZ */
		ACCEL_DLPF_5_1_HZ = 6,    /**< ACCEL_DLPF_5_1_HZ */
		ACCEL_DLPF_420_HZ = 7,    /**< ACCEL_DLPF_420_HZ */
		ACCEL_DLPF_BYPASS_1046_HZ,/**< ACCEL_DLPF_BYPASS_1046_HZ */
	} icm20602_accel_dlpf_t;

	/**
	 * Full scale acceleration range.
	 */
	typedef enum {
		ACCEL_FS_2G = 0, /**< Accel full scale range +-2g */
		ACCEL_FS_4G = 1, /**< Accel full scale range +-4g */
		ACCEL_FS_8G = 2, /**< Accel full scale range +-8g */
		ACCEL_FS_16G = 3,/**< Accel full scale range +-16g */
	} icm20602_accel_fs_t;

	/**
	 * Configuration options for the gyroscope digital low pass filter in the IMU.
	 */
	typedef enum {
		GYRO_DLPF_250_HZ = 0, // data clocked at 8kHz
		GYRO_DLPF_176_HZ = 1, // data clocked at 1kHz
		GYRO_DLPF_92_HZ = 2, // data clocked at 1kHz
		GYRO_DLPF_41_HZ = 3, // data clocked at 1kHz
		GYRO_DLPF_20_HZ = 4, // data clocked at 1kHz
		GYRO_DLPF_10_HZ = 5, // data clocked at 1kHz
		GYRO_DLPF_5_HZ = 6, // data clocked at 1kHz
		GYRO_DLPF_3281_HZ = 7, // data clocked at 8kHz
		GYRO_DLPF_BYPASS_3281_HZ, // no filter, data clocked at 32kHz
		GYRO_DLPF_BYPASS_8173_HZ, // no filter, data clocked at 32kHz
	} icm20602_gyro_dlpf_t;

	/**
	 * Gyroscope range.
	 */
	typedef enum {
		GYRO_RANGE_250_DPS = 0, /**< GYRO_RANGE_250_DPS */
		GYRO_RANGE_500_DPS = 1, /**< GYRO_RANGE_500_DPS */
		GYRO_RANGE_1000_DPS = 2,/**< GYRO_RANGE_1000_DPS */
		GYRO_RANGE_2000_DPS = 3,/**< GYRO_RANGE_2000_DPS */
	} icm20602_gyro_dps_t;

	/**
	 * Register map of the ICM20602
	 */
	typedef enum {
		REG_SMPLRT_DIV = 0x19,    /**< REG_SMPLRT_DIV */
		REG_CONFIG = 0x1A,        /**< REG_CONFIG */
		REG_GYRO_CONFIG = 0x1B,   /**< REG_GYRO_CONFIG */
		REG_ACCEL_CONFIG = 0x1C,  /**< REG_ACCEL_CONFIG */
		REG_ACCEL_CONFIG_2 = 0x1D,/**< REG_ACCEL_CONFIG_2 */
		REG_ACCEL_XOUT_H = 0x3B,  /**< REG_ACCEL_XOUT_H */
		REG_USER_CTRL = 0x6A,     /**< REG_USER_CTRL */
		REG_PWR_MGMT_1 = 0x6B,    /**< REG_PWR_MGMT_1 */
		REG_PWR_MGMT_2 = 0x6C,    /**< REG_PWR_MGMT_2 */
		REG_I2C_IF = 0x70,        /**< REG_I2C_IF */
		REG_FIFO_COUNTH = 0x72,   /**< REG_FIFO_COUNTH */
		REG_FIFO_COUNTL = 0x73,   /**< REG_FIFO_COUNTL */
		REG_FIFO_R_W_ = 0x74,     /**< REG_FIFO_R_W_ */
		REG_WHO_AM_I = 0x75,      /**< REG_WHO_AM_I */
	} icm20602_reg_t;

	/**
	 * Struct that holds all user configuration options
	 */
	typedef struct config_t {
		icm20602_accel_dlpf_t accelDlpf;
		icm20602_accel_fs_t accelFs;
		icm20602_gyro_dlpf_t gyroDlpf;
		icm20602_gyro_dps_t gyroDps;
		uint8_t sampleRateDiv;
	} config_t;


	ICM20602(const config_t& cfg, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin);
	virtual ~ICM20602();

	/**
	 * Start and configure the IMU
	 * @return True if IMU was configured successful. False if there was an error or no connection possible.
	 */
	bool start();

	/**
	 * Read the data (acceleration, gyro and temperature) from the IMU and update data variables
	 * in a synchronus way (meaning that the thread will yield during the request).
	 * @return True if read was successful
	 */
	bool read();

	/**
	 * Zero all Gyro and Accel values! This can be only called when the IMU is still!!
	 * TODO Z zero???
	 */
	void resetValues();

	/**
	 * Get acceleration in X direction of the IMU in m/s^2.
	 * @return Acceleration in X direction of the IMU in m/s^2.
	 */
	inline float getAccelX() { return m_accelX - m_accelXOffset; }

	/**
	 * Get acceleration in Y direction of the IMU in m/s^2.
	 * @return Acceleration in Y direction of the IMU in m/s^2.
	 */
	inline float getAccelY() { return m_accelY - m_accelYOffset; }

	/**
	 * Get acceleration in Z direction of the IMU in m/s^2.
	 * @return Acceleration in Z direction of the IMU in m/s^2.
	 */
	inline float getAccelZ() { return m_accelZ - m_accelZOffset; }

	/**
	 * Get the internal temperature of the IMU in °C.
	 * @return Internal temperature in °C.
	 */
	inline float getTemperature() { return m_temperature; }

	/**
	 * Get rotation around X axis of the IMU in °/s.
	 * @return Rotation around X axis in °/s.
	 */
	inline float getGyroX() { return m_gyroX - m_gyroXOffset; }

	/**
	 * Get rotation around Y axis of the IMU in °/s.
	 * @return Rotation around Y axis in °/s.
	 */
	inline float getGyroY() { return m_gyroY - m_gyroYOffset; }

	/**
	 * Get rotation around Z axis of the IMU in °/s.
	 * @return Rotation around Z axis in °/s.
	 */
	inline float getGyroZ() { return m_gyroZ - m_gyroZOffset; }

private:
	const config_t m_cfg;
	SPI_HandleTypeDef* m_hspi;
	GPIO_TypeDef* m_csPort;
	uint16_t m_csPin;
	bool m_initDone = false;

	uint8_t m_data[14];

	float m_accelSensitivity = 0.0f;
	float m_gyroSensitivity = 0.0f;

	constexpr static float GRAVITATIONAL_ACCELERATION = 9.81f;

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
	float m_accelXOffset = 0.0f;
	float m_accelYOffset = 0.0f;
	float m_accelZOffset = 0.0f;
	float m_gyroXOffset = 0.0f;
	float m_gyroYOffset = 0.0f;
	float m_gyroZOffset = 0.0f;

	/**
	 * Enable the IMU, so pull the CS low.
	 */
	inline void csEnable();

	/**
	 * Disable the IMU, so pull the CS high.
	 */
	inline void csDisable();

	/**
	 * Write a register
	 * @param reg 8-bit register address
	 * @param val 8-bit register value
	 */
	void writeReg(uint8_t reg, uint8_t val);

	/**
	 * Read a register
	 * @param reg 8-bit register address
	 * @return Value of the register
	 */
	uint8_t readReg(uint8_t reg);
};

#endif /* INCLUDE_ICM20602_HPP_ */
