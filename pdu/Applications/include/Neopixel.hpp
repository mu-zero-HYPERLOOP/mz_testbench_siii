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
	uint8_t r;
	uint8_t g;
	uint8_t b;

	constexpr color_t(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}
};

void init(TIM_HandleTypeDef* timer, uint16_t channel, uint16_t numberOfLeds);

void dispose();

void set(uint16_t index, color_t color);

void set(uint16_t start, uint16_t end, color_t color);

void setAll(color_t color);

void snapshot(color_t* snapshot);

void update();

}
