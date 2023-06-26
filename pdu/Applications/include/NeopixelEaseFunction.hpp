#pragma once

namespace neopixel {

class EaseFunction {
public:
	virtual ~EaseFunction() = default;
	virtual float apply(float time) = 0;

};

}
