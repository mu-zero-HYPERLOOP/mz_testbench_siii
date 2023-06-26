/*
 * NeopixelAnimation.hpp
 *
 *  Created on: Jun 26, 2023
 *      Author: karl
 */

#pragma once

#include <chrono>
#include "NeopixelFunction.hpp"
#include "NeopixelEaseFunction.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

namespace neopixel {

template<typename Function, typename EaseFunction>
class Animation {
public:

	Animation(uint16_t start, uint16_t end, std::chrono::duration<float> duration) :
		m_start(start),
		m_end(end),
		m_duration(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()),
		m_function{}, m_easeFunction{}, m_startTick(0) {
		if(m_start > m_end){
			uint16_t tmp = m_start;
			m_start = m_end;
			m_end = tmp;
		}
	}

	virtual ~Animation() = default;

	bool tick(){
		if(m_startTick == 0){
			m_startTick = xTaskGetTickCount();
		}
		TickType_t ticksSinceStart = xTaskGetTickCount() - m_startTick;
		float time = ticksSinceStart / m_duration;
		time = m_easeFunction.apply(time);
		for(uint16_t u16pos = m_start; u16pos <= m_end; u16pos++){
			const float pos = u16pos / (m_end - m_start);
			const color_t color = m_function->apply(pos, time);
			neopixel::set(u16pos, color);
		}

		return (ticksSinceStart >= m_duration);
	}
	void reset() {
		m_start = 0;
	}

private:
	uint16_t m_start;
	uint16_t m_end;
	TickType_t m_duration;
    Function m_function;
	EaseFunction m_easeFunction;

	TickType_t m_startTick;
};

}
