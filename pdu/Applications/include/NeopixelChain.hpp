/*
 * NeopixelChain.hpp
 *
 *  Created on: Jun 26, 2023
 *      Author: karl
 */

#pragma once

namespace neopixel {


template<typename OuterFunc, typename InnerFunc>
class chain {

	float apply(float pos, float time){
		return OuterFunc(InnerFunc(pos, time));
	}

};

}
