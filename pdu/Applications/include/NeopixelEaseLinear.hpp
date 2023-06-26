/*
 * NeopixelEaseLinear.hpp
 *
 *  Created on: Jun 26, 2023
 *      Author: karl
 */

#pragma once

#include "NeopixelEaseFunction.hpp"

namespace neopixel {

class ease_linear : public EaseFunction{
	float apply(float time) {
		return time;
	}
};

}
