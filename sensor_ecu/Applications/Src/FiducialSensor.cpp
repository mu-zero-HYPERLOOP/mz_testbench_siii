/*
 * FiducialSensor.cpp
 *
 *  Created on: Apr 25, 2023
 *      Author: OfficeLaptop
 */
#include "FiducialSensor.hpp"
#include "estdio.hpp"

FiducialSensor::FiducialSensor(GPIO_TypeDef* port, uint16_t pin, TIM_HandleTypeDef* htim, float distanceBetweenInterrupts)
		: m_exti(port, pin), m_count(0), m_timer(htim), m_deltaTime(0), m_distanceBetweenInterrupts(distanceBetweenInterrupts) {
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

