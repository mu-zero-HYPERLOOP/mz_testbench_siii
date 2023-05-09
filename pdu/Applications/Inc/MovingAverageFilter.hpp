/*
 * MovingAverageFilter.hpp
 *
 *  Created on: May 8, 2023
 *      Author: OfficeLaptop
 */

#pragma once

class MovingAverageFilter {

public:
	explicit MovingAverageFilter(float alpha = 0.1){
	}

	float update(float value){
		if(m_notInitalized){
			m_movingAverage = value;
		}else{
			m_movingAverage = (m_movingAverage + value) * m_alpha;
		}
		return m_movingAverage;
	}

	float get(){
		return m_movingAverage;
	}


private:
	float m_alpha;
	bool m_notInitalized = true;
	float m_movingAverage = 0.0;
};
