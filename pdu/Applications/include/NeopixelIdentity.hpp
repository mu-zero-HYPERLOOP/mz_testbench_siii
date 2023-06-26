/*
 * NeopixelIdentity.hpp
 *
 *  Created on: Jun 26, 2023
 *      Author: karl
 */

#pragma once

#include "NeopixelFunction.hpp"

namespace neopixel {

class pos_identity{
public:
	float apply(float pos, float time){
		return pos;
	}
};

class time_identity{
public:
	float apply(float pos, float time){
		return time;
	}
};

}
