/*
 * NeopixelMix.hpp
 *
 *  Created on: Jun 26, 2023
 *      Author: karl
 */

#pragma once

#include "NeopixelFunction.hpp"

namespace neopixel {


template<color_t COLOR_A, color_t COLOR_B, typename Func>
class mix : public Function{
public:
	mix() : m_func{} {};
	color_t apply(float time, float pos) override{
		float value = m_func.apply(time, pos);
		if(value >= 1)return COLOR_B;
		if(value <= 0)return COLOR_A;
		return color_t(
				COLOR_A.r * value + COLOR_B.r * (1-value),
				COLOR_A.g * value + COLOR_B.g * (1-value),
				COLOR_A.b * value + COLOR_B.b * (1-value)
				);

	}
private:
	Func m_func;
};

}
