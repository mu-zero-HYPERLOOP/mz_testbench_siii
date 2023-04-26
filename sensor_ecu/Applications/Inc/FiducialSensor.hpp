/*
 * FiducialSensor.hpp
 *
 *  Created on: Apr 25, 2023
 *      Author: OfficeLaptop
 */
#pragma once

#include "gpio.h"
#include <cinttypes>
#include "GPIOExtiController.hpp"
#include "Timer.hpp"
#include "peripheral_config.hpp"

class FiducialSensor {
public:
	explicit FiducialSensor(FiducialConfig config);
	~FiducialSensor();

	void reset();

	void extiCallback(bool v);

	[[nodiscard]] float estimateVelocityMPS();

	[[nodiscard]] inline unsigned int getCount() {
		return m_count;
	}

	[[nodiscard]] inline uint32_t getDeltaTime(){
		return m_deltaTime;
	}


	[[nodiscard]] inline bool current(){
		return m_exti.read();
	}


private:
	GPIOExtiController m_exti;
	unsigned int m_count;
	Timer m_timer;
	uint32_t m_deltaTime;
	uint32_t m_distanceBetweenInterrupts;
};

