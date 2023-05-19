/*
 * MovingAverageFilter.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include <cinttypes>

template<size_t N, typename T = float>
class MovingAverageFilter {
public:
	explicit MovingAverageFilter(T inital){
		for(size_t i=0;i<N;i++){
			m_buf[i] = inital / N;
		}
		m_average = inital;
		m_idx = 0;
	}

	void addValue(const T& value){
		m_average -= m_buf[m_idx];
		m_buf[m_idx] = value / N;
		m_average += m_buf[m_idx];
		m_idx = (m_idx + 1) % N;
	}

	T get(){
		return m_average;
	}

private:
	T m_buf[N];
	T m_average;
	size_t m_idx;
};
