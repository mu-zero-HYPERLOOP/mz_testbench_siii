/*
 * LocalSensorReadings.hpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include <cinttypes>
#include "canzero.hpp"

class SensorValueRegistry {
public:
	static SensorValueRegistry& getInstance() {
		static SensorValueRegistry instance;
		return instance;
	}

	void updateCAN(){
		can::Message<can::messages::SensorF_TX_AccFront> imuAccelMsg;
		imuAccelMsg.set<can::signals::SensorF_AccFront_X>(m_accelX);
		imuAccelMsg.set<can::signals::SensorF_AccFront_Y>(m_accelY);
		imuAccelMsg.set<can::signals::SensorF_AccFront_Z>(m_accelZ);
		imuAccelMsg.set<can::signals::SensorF_GyroFront_Z>(m_gyroZ);
		imuAccelMsg.send();
	}

	void setGyro(float x, float y, float z){
		m_gyroX = x;
		m_gyroY = y;
		m_gyroZ = z;
	}

	void setAccel(float x, float y, float z){
		m_accelX = x;
		m_accelY = y;
		m_accelZ = z;
	}

	void setCoolingReservoirTemperatur(float temperaturC){
		m_coolingReservoirTemperatur = temperaturC;
	}

	void setCoolingPressure(float pressure){
		m_coolingPressure = pressure;
	}

	void setFiducialLeftValues(unsigned int counter, uint32_t dt, float vel, float pos){
		m_fiducialLeftCounter = counter;
		m_fiducialLeftDeltaTime = dt;
		m_fiducialLeftVelocity = vel;
		m_fiducialLeftPos = pos;
	}

	void setFiducialRightValues(unsigned int counter, uint32_t dt, float vel, float pos){
		m_fiducialRightCounter = counter;
		m_fiducialRightDeltaTime = dt;
		m_fiducialRightVelocity = vel;
		m_fiducialRightPos = pos;
	}

	void setKistlerValues(float vel, float pos){
		m_kistlerVel = vel;
		m_kistlerPos = pos;
	}

	[[nodiscard]] inline float getKistlerVelocity(){
		return m_kistlerVel;
	}

	[[nodiscard]] inline float getKistlerPosition(){
		return m_kistlerPos;
	}

	[[nodiscard]] inline float getCoolingReservoirTemperatur(){
		return m_coolingReservoirTemperatur;
	}

	[[nodiscard]] inline float getCoolingPressure(){
		return m_coolingPressure;
	}

	[[nodiscard]] inline float getImuGyroX(){
		return m_gyroX;
	}

	[[nodiscard]] inline float getImuGyroY(){
		return m_gyroY;
	}

	[[nodiscard]] inline float getImuGyroZ(){
		return m_gyroZ;
	}

	[[nodiscard]] inline float getImuAccelX(){
		return m_accelX;
	}

	[[nodiscard]] inline float getImuAccelY(){
		return m_accelY;
	}

	[[nodiscard]] inline float getImuAccelZ(){
		return m_accelZ;
	}

	[[nodiscard]] inline unsigned int getFiducialLeftCount(){
		return m_fiducialLeftCounter;
	}

	[[nodiscard]] inline float getFiducialLeftVelocity(){
		return m_fiducialLeftVelocity;
	}

	[[nodiscard]] inline float getFiducialLeftPosition(){
		return m_fiducialLeftPos;
	}

	[[nodiscard]] inline unsigned int getFiducialRightCounter(){
		return m_fiducialRightCounter;
	}

	[[nodiscard]] inline float getFiducialRightVelocity(){
		return m_fiducialRightVelocity;
	}

	[[nodiscard]] inline float getFiducialRightPosition(){
		return m_fiducialRightPos;
	}

private:
	explicit SensorValueRegistry(){

	}
	SensorValueRegistry(SensorValueRegistry&) = delete;
	SensorValueRegistry(SensorValueRegistry&&) = delete;
	SensorValueRegistry& operator=(SensorValueRegistry&) = delete;
	SensorValueRegistry& operator=(SensorValueRegistry&&) = delete;

private:
	float m_gyroX;
	float m_gyroY;
	float m_gyroZ;
	float m_accelX;
	float m_accelY;
	float m_accelZ;

	float m_coolingReservoirTemperatur;

	float m_coolingPressure;

	unsigned int m_fiducialLeftCounter;
	uint32_t m_fiducialLeftDeltaTime;
	float m_fiducialLeftVelocity;
	float m_fiducialLeftPos;

	unsigned int m_fiducialRightCounter;
	uint32_t m_fiducialRightDeltaTime;
	float m_fiducialRightVelocity;
	float m_fiducialRightPos;

	float m_kistlerVel;
	float m_kistlerPos;



};
