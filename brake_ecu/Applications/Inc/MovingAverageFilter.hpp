/*
 * MovingAverageFilter.hpp
 *
 *  Created on: May 14, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <cinttypes>
#include <functional>

template<size_t N>
class MovingAverageFilter {
public:
	explicit MovingAverageFilter() : m_mutex(osMutexNew(NULL)){

	}
	~MovingAverageFilter(){
		osMutexDelete(m_mutex);
	}

	void refresh(std::function<float()> newElementFunction){
		osMutexAcquire(m_mutex, osWaitForever);
		float sum = 0;
		for(size_t i=0;i<N;i++){
			float value = newElementFunction();
			m_buffer[m_start++] = value;
			sum += value;
		}
		m_average = sum / N;
		osMutexRelease(m_mutex);
	}

	float average(){
		osMutexAcquire(m_mutex, osWaitForever);
		float average =  m_average;
		osMutexRelease(m_mutex);
		return average;
	}

	void push(float value){
		osMutexAcquire(m_mutex, osWaitForever);
		m_average -= m_buffer[m_start];
		m_buffer[m_start] = value / N;
		m_average += m_buffer[m_start];
		osMutexRelease(m_mutex);
	}

private:
	float m_average = 0;
	float m_buffer[N] = {};
	size_t m_start = 0;
	osMutexId_t m_mutex;
};
