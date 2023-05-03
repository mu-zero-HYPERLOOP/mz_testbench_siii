/*
 * GlobalSensorRegister.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */
#pragma once

#include "FiducialSensor.hpp"
#include "SDC.hpp"
#include "ImuMaster.hpp"
#include "NTCSensor.hpp"
#include "PressureSensor.hpp"
#include "KistlerController.hpp"
#include "peripheral_config.hpp"

class GlobalSensorRegister {
private:
	GlobalSensorRegister() :
			m_fiducialSensorRight(g_peripherals.m_fiducialRightConfig), m_fiducialSensorLeft(
					g_peripherals.m_fiducialLeftConfig), m_coolingReservoirTemperatur(
					g_peripherals.m_coolingReservoirTemperaturSensorConfig), m_coolingPressure(), m_kistlerController() {

	}
	GlobalSensorRegister(GlobalSensorRegister&) = delete;
	GlobalSensorRegister(GlobalSensorRegister&&) = delete;
	GlobalSensorRegister& operator=(GlobalSensorRegister&) = delete;
	GlobalSensorRegister& operator=(GlobalSensorRegister&&) = delete;
public:
	[[nodiscard]] static inline GlobalSensorRegister& getInstance() {
		static GlobalSensorRegister instance;
		return instance;
	}

	[[nodiscard]] inline FiducialSensor& getFiducialSensorRight(){
		return m_fiducialSensorRight;
	}

	[[nodiscard]] inline FiducialSensor& getFiducialSensorLeft(){
		return m_fiducialSensorLeft;
	}

	[[nodiscard]] inline NTCSensor& getCoolingReservoirTemperaturSensor(){
		return m_coolingReservoirTemperaturSensor;
	}

	[[nodiscard]] inline PressureSensor& getCoolingPressureSensor() {
		return m_coolingPressure;
	}

	[[nodiscard]] inline KistlerController& getKistlerController(){
		return m_kistlerController;
	}

private:
	FiducialSensor m_fiducialSensorRight;
	FiducialSensor m_fiducialSensorLeft;
	NTCSensor m_coolingReservoirTemperatur;
	PressureSensor m_coolingPressure;
	KistlerController m_kistlerController;
};

