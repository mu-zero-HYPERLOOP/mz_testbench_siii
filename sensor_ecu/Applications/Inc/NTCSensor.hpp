/*
 * NTCSensor.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "AnalogInput.hpp"
#include "AdcModule.hpp"
#include <cinttypes>
#include <cmath>


class NTCSensor {
public:
	explicit NTCSensor(AdcModule module, uint16_t rank, float internalResistance, float supplyVoltage, float beta, float r25)
		: m_channelController(module, rank),
		  m_r(internalResistance),
		  m_u0(supplyVoltage),
		  m_beta(beta),
		  m_r25(r25){
	}

	float getTemperaturC(bool force = false){
		return getTemperaturK(force) - t0;
	}

	float helper(uint16_t avalue, float r){

		float r_ntc = ((4095 * r) / (avalue )) - r;
		float log = std::log(r_ntc / m_r25);
		float denom = log / m_beta + t25_inv;
		return 1.0 / denom;
	}

	float getTemperaturK(bool force = false){
		uint16_t avalue = m_channelController.get(force);
		float min = 100000;
		float bestR = 0;
		float target = 273.15 + 14;
		for(float r = 1000;r<100000;r+= 100){
			float temp = helper(avalue, r);
			if(std::abs(temp - target) < min){
				min = std::abs(temp - target);
				bestR = r;
			}
		}
		printf("best_r = %f\n", bestR);
		return helper(avalue, bestR);
	}

	float configure(float temperatureK){
		uint16_t avalue = m_channelController.get(true);
		float min = 100000;
		float bestR = 0;
		float target =  temperatureK;
		for(float r = 1000;r<100000;r+= 100){
			float temp = helper(avalue, r);
			if(std::abs(temp - target) < min){
				min = std::abs(temp - target);
				bestR = r;
			}
		}
		m_r = bestR;
		printf("new resistance value = %f\n", bestR);
		return helper(avalue, bestR);
	}

	void setR(float r){
		m_r = r;
	}

private:
	AdcChannelController m_channelController;
	float m_r;
	float m_u0;
	float m_beta;
	float m_r25;
	static constexpr float t25_inv = 1.0 / 298.15;
	static constexpr float t0 = 273.15;

};
