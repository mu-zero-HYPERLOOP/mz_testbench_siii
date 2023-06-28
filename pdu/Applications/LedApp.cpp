/*
 * LedApp.cpp
 *
 *  Created on: Jun 26, 2023
 *      Author: karl
 */
#include "Neopixel.hpp"
#include "estdio.hpp"
#include <chrono>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

void led_app_entry(void* argv) {
	using namespace neopixel;
	using namespace std::chrono;
	using namespace std::chrono_literals;

	constexpr uint32_t ledCount = 108 * 2 - 1;

	neopixel::init(&htim3, TIM_CHANNEL_1, ledCount);
	neopixel::setAll(color_t(255,0,255));

	bool switchero = false;
	uint32_t count = 0;
	while(true){
		neopixel::set(0, count, color_t(123,0,0));
		neopixel::set(count + 1, ledCount, color_t(10, 50, 23));

		neopixel::update();
		osDelay(pdMS_TO_TICKS(50));
		switchero = !switchero;
		count++;
		if(count >= ledCount){
			count = 0;
		}
	}
}
