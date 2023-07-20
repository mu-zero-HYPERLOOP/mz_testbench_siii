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
#include "GlobalState.hpp"
#include <cmath>

static state::PodState lastState = state::STATE::POD_STARTUP;
static TickType_t lastTransition = 0;

static constexpr TickType_t startupAnimationDuration = pdMS_TO_TICKS(5000);
static constexpr float startupAnimationFadeStrength = 2.0;

static constexpr neopixel::color_t IDLE_COLOR_DARK = neopixel::color_t(0,0xAA/16,0x88/16);
static constexpr neopixel::color_t IDLE_COLOR_BRIGHT = neopixel::color_t(0x00, 0xAA/2, 0x88/2);

static constexpr uint16_t LEFT_LED_START = 0;
static constexpr uint16_t LEFT_LED_END = 107;
static constexpr uint16_t LEFT_LED_COUNT = 215;
static constexpr uint16_t RIGHT_LED_START = 108;
static constexpr uint16_t RIGHT_LED_END = 108;
static constexpr uint16_t RIGHT_LED_COUNT = 108;

static constexpr float IDLE_BREATH_SIZE = 0.75f;
static constexpr float IDLE_BREATH_STRENGTH = 3;
static constexpr float IDLE_BREATH_BLEND = 4;

constexpr neopixel::color_t MU_ZERO_COLOR = neopixel::color_t(0, 255, 0xDD);

static neopixel::color_t learp(neopixel::color_t a, neopixel::color_t b, float alpha){
	if(alpha >= 1){
		return a;
	}
	if(alpha <= 0){
		return b;
	}
	return neopixel::color_t(
		a.r * alpha + b.r * (1-alpha),
		a.g * alpha + b.g * (1-alpha),
		a.b * alpha + b.b * (1-alpha)
	);
}

static float ease_in_out(float x, float p){
	if(x < 0.5){
		return std::pow(2.0f, p-1.0f) * std::pow(x, p);
	}else{
		return 1.0f - std::pow(-2.0f*x + 2.0f, p)/2.0f;
	}
}

static float ease_out(float x, float p){
	return 1.0f - std::pow(1.0f - x, p);
}


static neopixel::color_t* colorSnapshot = nullptr;

void led_state_maschine(){
	state::PodState state = state::get();
	bool transition = false;
	if(state != lastState){
		//lastTransition = xTaskGetTickCount();
		transition = true;
	}

	TickType_t timeSinceLastTransition = xTaskGetTickCount() - lastTransition;

	switch(state){
	case state::STATE::POD_STARTUP:
	{
		if(transition){
			//neopixel::snapshot(colorSnapshot);
		}
		float time = timeSinceLastTransition / (float) startupAnimationDuration;

		if(time < 1){
			time = ease_in_out(time, 3);
			if(LEFT_LED_START < LEFT_LED_END){
				for(uint16_t pixel = LEFT_LED_START; pixel <= LEFT_LED_END; pixel++){
					float pos = (pixel - LEFT_LED_START) /(float) (LEFT_LED_COUNT);

					float alpha = 1.0f - 1.0f /
							(1.0f + std::exp(-(
									(pos-time*3+1)*4*startupAnimationFadeStrength
									)));
					const neopixel::color_t& refColor = colorSnapshot[pixel];
					const neopixel::color_t& destColor = IDLE_COLOR_DARK;
					neopixel::set(pixel, learp(destColor, refColor, alpha));
				}

			}else{
				for(uint16_t pixel = LEFT_LED_END; pixel <= LEFT_LED_START; pixel++){
					float pos = 1.0f - (pixel - LEFT_LED_END) /(float) (LEFT_LED_COUNT);

					float alpha = 1.0f - 1.0f /
							(1.0f + std::exp(-(
									(pos-time*3+1)*4*startupAnimationFadeStrength
									)));
					const neopixel::color_t& refColor = colorSnapshot[pixel];
					const neopixel::color_t& destColor = IDLE_COLOR_DARK;
					neopixel::set(pixel, learp(destColor, refColor, alpha));
				}

			}
			if(RIGHT_LED_START < RIGHT_LED_END){
				for(uint16_t pixel = RIGHT_LED_START; pixel <= RIGHT_LED_END; pixel++){
					float pos = (pixel - RIGHT_LED_START) /(float) (RIGHT_LED_COUNT);

					float alpha = 1.0f - 1.0f /
							(1.0f + std::exp(-(
									(pos-time*3+1)*4*startupAnimationFadeStrength
									)));
					const neopixel::color_t& refColor = colorSnapshot[pixel];
					const neopixel::color_t& destColor = IDLE_COLOR_DARK;
					neopixel::set(pixel, learp(destColor, refColor, alpha));
				}
			}else{
				for(uint16_t pixel = RIGHT_LED_END; pixel <= RIGHT_LED_START; pixel++){
					float pos = 1.0f - (pixel - RIGHT_LED_END) /(float) (RIGHT_LED_COUNT);

					float alpha = 1.0f - 1.0f /
							(1.0f + std::exp(-(
									(pos-time*3+1)*4*startupAnimationFadeStrength
									)));
					const neopixel::color_t& refColor = colorSnapshot[pixel];
					const neopixel::color_t& destColor = IDLE_COLOR_DARK;
					neopixel::set(pixel, learp(destColor, refColor, alpha));
				}
			}
		}else{ //time >= 1.0f
			time -= 1.0f; //subtract startup Animation time.
			time = std::fmod(time, 2.0f);
			if(time > 1.0f){
				time = 2.0f - time;
			}
			time = std::max(0.35f,ease_out(time * 0.8 + 0.05, IDLE_BREATH_STRENGTH));

			if(LEFT_LED_START < LEFT_LED_END){
				uint16_t start = LEFT_LED_START + LEFT_LED_COUNT - LEFT_LED_COUNT * IDLE_BREATH_SIZE;
				for(uint16_t pixel = start;
						pixel <= LEFT_LED_END; pixel++){
					float pos = 1.0f - (pixel - start) /(float) (IDLE_BREATH_SIZE * LEFT_LED_COUNT);

					float alpha = 1.0f - 1.0f /
							(1.0f + std::exp(-(
									(pos +0.25-time)*16
									)));
					const neopixel::color_t& refColor = IDLE_COLOR_DARK;
					const neopixel::color_t& destColor = IDLE_COLOR_BRIGHT;
					neopixel::set(pixel, learp(destColor, refColor, alpha));
				}

			}else{
				for(uint16_t pixel = LEFT_LED_END; pixel <= LEFT_LED_END +
				static_cast<uint16_t>(LEFT_LED_COUNT * IDLE_BREATH_SIZE); pixel++){
					float pos = (pixel - LEFT_LED_END) /(float) (IDLE_BREATH_SIZE * LEFT_LED_COUNT);

					float alpha = 1.0f - 1.0f /
							(1.0f + std::exp(-(
									(pos +0.25-time)*16
									)));
					const neopixel::color_t& refColor = IDLE_COLOR_DARK;
					const neopixel::color_t& destColor = IDLE_COLOR_BRIGHT;
					neopixel::set(pixel, learp(destColor, refColor, alpha));
				}

			}
			if(RIGHT_LED_START < RIGHT_LED_END){
				uint16_t start = RIGHT_LED_START + RIGHT_LED_COUNT - RIGHT_LED_COUNT * IDLE_BREATH_SIZE;
				for(uint16_t pixel = start;
						pixel <= RIGHT_LED_END; pixel++){
					float pos = 1.0f - (pixel - start) /(float) (IDLE_BREATH_SIZE * RIGHT_LED_COUNT);

					float alpha = 1.0f - 1.0f /
							(1.0f + std::exp(-(
									(pos +0.25-time)*16
									)));
					const neopixel::color_t& refColor = IDLE_COLOR_DARK;
					const neopixel::color_t& destColor = IDLE_COLOR_BRIGHT;
					neopixel::set(pixel, learp(destColor, refColor, alpha));
				}

			}else{
				for(uint16_t pixel = RIGHT_LED_END; pixel <= RIGHT_LED_END +
				static_cast<uint16_t>(RIGHT_LED_COUNT * IDLE_BREATH_SIZE); pixel++){
					float pos = (pixel - RIGHT_LED_END) /(float) (IDLE_BREATH_SIZE * RIGHT_LED_COUNT);

					float alpha = 1.0f - 1.0f /
							(1.0f + std::exp(-(
									(pos +0.25-time)*16
									)));
					const neopixel::color_t& refColor = IDLE_COLOR_DARK;
					const neopixel::color_t& destColor = IDLE_COLOR_BRIGHT;
					neopixel::set(pixel, learp(destColor, refColor, alpha));
				}
			}

			/*
			float intensity = std::cos(time * 2*M_PI + M_PI) * 0.5f + 0.5f;
			const neopixel::color_t& lowColor = IDLE_COLOR_DARK;
			const neopixel::color_t& highColor = IDLE_COLOR_BRIGHT;
			const neopixel::color_t color = learp(highColor, lowColor, intensity);
			{
				uint16_t leftStart = std::min(LEFT_LED_START, LEFT_LED_END);
				uint16_t leftEnd = std::max(LEFT_LED_START, LEFT_LED_END);
				for(uint16_t pixel = leftStart; pixel <= leftEnd; pixel++){
					neopixel::set(pixel, color);
				}
			}
			{
				uint16_t rightStart = std::min(RIGHT_LED_START, RIGHT_LED_END);
				uint16_t rightEnd = std::max(RIGHT_LED_START, RIGHT_LED_END);
				for(uint16_t pixel = rightStart; pixel <= rightEnd; pixel++){
					neopixel::set(pixel, color);
				}
			}
			*/
		}


		break;
	}

	}


}

void led_app_entry(void* argv) {

	constexpr uint32_t ledCount = 108 * 2 - 1;

	neopixel::init(&htim3, TIM_CHANNEL_1, ledCount);
	neopixel::setAll(neopixel::color_t(0,0,0));

	colorSnapshot = reinterpret_cast<neopixel::color_t*>(calloc(ledCount, sizeof(neopixel::color_t)));
	neopixel::snapshot(colorSnapshot);
	lastTransition = xTaskGetTickCount();

	while(true){
		led_state_maschine();
		neopixel::update();
		osDelay(pdMS_TO_TICKS(50));
	}
}
