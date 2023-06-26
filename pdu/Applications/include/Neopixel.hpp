/*
 * Neopixel.hpp
 *
 *  Created on: Jun 26, 2023
 *      Author: karl
 */

#pragma once
#include <cinttypes>
#include "tim.h"


namespace neopixel {

struct color_t {
	const uint8_t r;
	const uint8_t g;
	const uint8_t b;

	constexpr color_t(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
	constexpr color_t(float r, float g, float b) : r(r * 255), g(g * 255), b(b * 255) {}
};

void init(TIM_HandleTypeDef* timer, uint16_t channel, uint16_t numberOfLeds);

void dispose();

void set(uint16_t index, color_t color);

void set(uint16_t start, uint16_t end, color_t color);

void update();

}
