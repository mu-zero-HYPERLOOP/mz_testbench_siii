/*
 * ICM20602.cpp
 *
 *  Created on: May 30, 2021
 *      Author: Flo Keck
 */

#include <ICM20602.hpp>
#include "FreeRTOS.h"
#include "cmsis_os2.h"

ICM20602::ICM20602(const config_t& cfg, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin) : m_cfg{cfg}, m_hspi{hspi}, m_csPort{csPort}, m_csPin{csPin} {

}

ICM20602::~ICM20602() {

}

inline void ICM20602::csEnable() {
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
}
inline void ICM20602::csDisable() {
	HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}

void ICM20602::writeReg(uint8_t reg, uint8_t val) {
	csEnable();
	HAL_SPI_Transmit(m_hspi, &reg, 1, 10);
	HAL_SPI_Transmit(m_hspi, &val, 1, 10);
	csDisable();
}

uint8_t ICM20602::readReg(uint8_t reg) {
	reg |= 128;		// Set read bit
	uint8_t val = 0;
	csEnable();
	HAL_SPI_Transmit(m_hspi, &reg, 1, 10);
	HAL_SPI_Receive(m_hspi, &val, 1, 10);
	csDisable();
	return val;
}

bool ICM20602::start() {

	writeReg(REG_I2C_IF, 0x40);

	// Reset device and wait
	writeReg(REG_PWR_MGMT_1, 0x80);
	osDelay(pdMS_TO_TICKS(10));

	// Check if connection is working by reading the WHO_AM_I register
	uint8_t whoAmI = readReg(REG_WHO_AM_I);
	if(whoAmI != 0x12) {
		return false;
	}

	// Set internal clock to PLL
	writeReg(REG_PWR_MGMT_1, 0x01);

	// Accel and gyro standby
	writeReg(REG_PWR_MGMT_2, 0x3F);

	// Disable FIFO
	writeReg(REG_USER_CTRL, 0x00);

	// Disable I2C interface
	writeReg(REG_I2C_IF, 0x40);

	// Enable and config acceleration
	if(m_cfg.accelDlpf == ACCEL_DLPF_BYPASS_1046_HZ) {
		writeReg(REG_ACCEL_CONFIG_2, (1 << 3));
	} else {
		writeReg(REG_ACCEL_CONFIG_2, m_cfg.accelDlpf);
	}
	writeReg(REG_ACCEL_CONFIG, m_cfg.accelFs << 3);
	if(m_cfg.accelFs == ACCEL_FS_2G) {
		m_accelSensitivity = 16384.0f;
	} else if(m_cfg.accelFs == ACCEL_FS_4G) {
		m_accelSensitivity = 8192.0f;
	} else if(m_cfg.accelFs == ACCEL_FS_8G) {
		m_accelSensitivity = 4096.0f;
	} else if(m_cfg.accelFs == ACCEL_FS_16G) {
		m_accelSensitivity = 2048.0f;
	} else {
		return false;
	}

	// Enable and config gyro
	if(m_cfg.gyroDlpf == GYRO_DLPF_BYPASS_3281_HZ) {
		// Bypass dpf and set dps
		writeReg(REG_CONFIG, 0x00);

		// See table page 37 of datasheet
		writeReg(REG_GYRO_CONFIG, (m_cfg.gyroDps << 3) | 0x02);
	} else if(m_cfg.gyroDlpf == GYRO_DLPF_BYPASS_8173_HZ) {
		// Bypass dpf and set dps
		writeReg(REG_CONFIG, 0x00);

		// See table page 37 of datasheet
		writeReg(REG_GYRO_CONFIG, (m_cfg.gyroDps << 3) | 0x01);
	} else {
		// Configure dpf and dps
		writeReg(REG_CONFIG, m_cfg.gyroDlpf);

		writeReg(REG_GYRO_CONFIG, m_cfg.gyroDps << 3);
	}
	if(m_cfg.gyroDps == GYRO_RANGE_250_DPS) {
		m_gyroSensitivity = 131.0f;
	} else if(m_cfg.gyroDps == GYRO_RANGE_500_DPS) {
		m_gyroSensitivity = 65.5f;
	} else if(m_cfg.gyroDps == GYRO_RANGE_1000_DPS) {
		m_gyroSensitivity = 32.8f;
	} else if(m_cfg.gyroDps == GYRO_RANGE_2000_DPS) {
		m_gyroSensitivity = 16.4f;
	} else {
		return false;
	}

	// We will not enable the FIFO

	// Configure sample rate divider
	writeReg(REG_SMPLRT_DIV, m_cfg.sampleRateDiv);

	// Enable both Accel and Gyro
	writeReg(REG_PWR_MGMT_2, 0x00);

	// Verify that connection is still working
	whoAmI = readReg(REG_WHO_AM_I);
	if(whoAmI != 0x12) {
		return false;
	}

	m_initDone = true;
	return true;
}


bool ICM20602::read() {
	if(!m_initDone) {
		return false;
	}

	// Read seven 16-bit data values, so 14 bytes: ACCEL_X ACCEL_Y ACCEL_Z TEMP GYRO_X GYRO_Y GYRO_Z
	csEnable();
	uint8_t reg = REG_ACCEL_XOUT_H | 128;
	HAL_SPI_Transmit(m_hspi, &reg, 1, 10);
	HAL_SPI_Receive(m_hspi, m_data, 14, 10);
	csDisable();

	// Get acceleration
	int16_t accelXRaw = (m_data[0] << 8) + m_data[1];
	int16_t accelYRaw = (m_data[2] << 8) + m_data[3];
	int16_t accelZRaw = (m_data[4] << 8) + m_data[5];
	m_accelX = accelXRaw / m_accelSensitivity * GRAVITATIONAL_ACCELERATION;
	m_accelY = accelYRaw / m_accelSensitivity * GRAVITATIONAL_ACCELERATION;
	m_accelZ = accelZRaw / m_accelSensitivity * GRAVITATIONAL_ACCELERATION;

	// Internal temperature sensor
	int16_t temperatureRaw = (m_data[6] << 8) + m_data[7];
	m_temperature = temperatureRaw / 326.8f + 25.0f;

	// Get gyroscope
	int16_t gyroXRaw = (m_data[8] << 8) + m_data[9];
	int16_t gyroYRaw = (m_data[10] << 8) + m_data[11];
	int16_t gyroZRaw = (m_data[12] << 8) + m_data[13];
	m_gyroX = gyroXRaw / m_gyroSensitivity;
	m_gyroY = gyroYRaw / m_gyroSensitivity;
	m_gyroZ = gyroZRaw / m_gyroSensitivity;

	return true;
}

void ICM20602::resetValues() {
	read();

	m_accelXOffset = m_accelX;
	m_accelYOffset = m_accelY;
	m_accelZOffset = m_accelZ;

	m_gyroXOffset = m_gyroX;
	m_gyroYOffset = m_gyroY;
	m_gyroZOffset = m_gyroZ;
}
