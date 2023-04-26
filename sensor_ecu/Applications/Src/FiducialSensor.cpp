/*
 * FiducialSensor.cpp
 *
 *  Created on: Apr 25, 2023
 *      Author: OfficeLaptop
 */
#include "FiducialSensor.hpp"
#include "estdio.hpp"

FiducialSensor::FiducialSensor(FiducialConfig config) :
		m_exti(config.m_gpio.m_port, config.m_gpio.m_pin), m_count(0), m_timer(config.m_htim), m_deltaTime(
				0), m_distanceBetweenInterrupts(
				config.m_distanceBetweenInterrupts) {
	m_exti.setExtiCallback([&](bool v) {
		this->extiCallback(v);
	});
	m_timer.start();
}

FiducialSensor::~FiducialSensor() {
	m_exti.resetExtiCallback();
}

void FiducialSensor::reset() {
	m_timer.reset();
}

float FiducialSensor::estimateVelocityMPS() {
	return m_distanceBetweenInterrupts / (float) m_deltaTime;
}

void FiducialSensor::extiCallback(bool v) {
	m_deltaTime = (m_timer.get() + m_timer.overflow() * 0xFFFF) / 10;
	m_timer.reset();
	m_count++;
}

