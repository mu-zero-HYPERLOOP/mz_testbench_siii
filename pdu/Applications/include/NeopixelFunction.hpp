#pragma once

namespace neopixel {

#include "Neopixel.hpp"

class Function {
public:
	virtual ~Function() = default;
	virtual color_t apply(float pos, float time) = 0;
};

}
