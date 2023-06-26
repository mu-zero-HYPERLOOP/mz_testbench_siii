/*
 * LedApp.cpp
 *
 *  Created on: Jun 26, 2023
 *      Author: karl
 */
#include "Neopixel.hpp"
#include "NeopixelAnimation.hpp"
#include "NeopixelMix.hpp"
#include "NeopixelEaseLinear.hpp"
#include "NeopixelIdentity.hpp"
#include <chrono>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

void led_app_entry(void* argv) {
	using namespace neopixel;
	using namespace std::chrono;
	using namespace std::chrono_literals;

	constexpr uint32_t ledCount = 200;

	neopixel::init(&htim3, TIM_CHANNEL_3, ledCount);

	/*
	neopixel::Animation<
		neopixel::mix<
			neopixel::color_t(1.0f, 0.0f, 1.0f),
			neopixel::color_t(0.0f, 1.0f, 0.0f),
			neopixel::pos_identity>,
		neopixel::ease_linear> animation1(0, 100, 10ms);
		*/

	bool switchero = false;
	while(true){

		for(uint32_t i = 0; i < ledCount; i++){
			if((bool)(i % 2) == switchero){
				neopixel::set(i, color_t(1.0f, 0.0f, 1.0f));
			}else{
				neopixel::set(i, color_t(0.0f, 1.0f, 0.0f));
			}
		}

		neopixel::update();
		osDelay(pdMS_TO_TICKS(50));
		switchero = !switchero;
	}
}
