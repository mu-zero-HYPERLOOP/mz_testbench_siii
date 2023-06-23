/*
 * ProjectXX.hpp
 *
 * Control the WS2812B LED strip around the shell of the pod known as "Project X".
 *
 * Sending data for 300 LEDs takes 9ms -> No Updates faster than 10ms
 *
 * Usage of WS2812B Driver:
 * 		Reset all LEDs to a color:
 * 			leds.setAll(COLORS::BLACK);
 * 		Set one led (Center LED at front of pod has index 0):
 * 			leds.set(ledIndex, COLORS:MU_GREEN, brightness = 0.5f)
 * 		Update LED strip (needs at least 10ms break between calls):
 * 			leds.update();
 *
 *  Created on: 05.05.2021
 *      Author: Flo Keck
 */

#ifndef INCLUDE_PROJECTXX_HPP_
#define INCLUDE_PROJECTXX_HPP_

#include "cz_interface.hpp"
#include "WS2812BDriver.hpp"
#include "ColorConversion.hpp"
#include <functional>

constexpr int16_t NUM_LEDS = 251 + 48; //63*2-1;	// Uneven number for index shift to center to work
WS2812BDriver<NUM_LEDS> leds { &htim3, TIM_CHANNEL_1 }; //LEDs are controlled by D1 (PB4)

constexpr float PI = 3.14159265359f;
constexpr float TWO_PI = 2 * PI;

using POD_STATE = messages::SensorF_TX_StatePod::SensorF_TX_PodState;
TickType_t lastStateChange = 0;
int8_t lastPodState = 1;	// Pod Idle state is default
int8_t podState = 0;
int8_t podTargetState = 0;

float podVelocity = 0;
float podPosition = 0;
float estimatedRunLength = 0;
float runProfileV0 = 0;
float runProfileV1 = 0;
float runProfileV2 = 0;
float runProfileV3 = 0;
float runProfileV4 = 0;
float runProfileVMax = 0;

namespace COLORS {
constexpr uint32_t BLACK = 0x000000;
constexpr uint32_t WHITE = 0xFFFFFF;
constexpr uint32_t RED = 0xFF0000;
constexpr uint32_t GREEN = 0x00FF00;
constexpr uint32_t BLUE = 0x0000FF;
constexpr uint32_t ORANGE = 0xFFAA00;
constexpr uint32_t PURPLE = 0x36013F;
constexpr uint32_t BRIGHT_PURPLE = 0xFF00FF;
constexpr uint32_t MU_GREEN = 0x00FFDD;	// KIT green, but with max value in HSV model
constexpr uint32_t KIT_GREEN = 0x009682;// The standard KIT green rgb(0, 150, 130)
constexpr uint32_t MU_RED = 0xFF0022;
constexpr uint32_t MU_ORANGE = 0xFF5E00;
constexpr uint32_t MU_PINK = 0xFC46AA;
}

/**
 * Macro that convertes a R, G, B triplet to a RGB hex value
 * @param r	Red value
 * @param g Green value
 * @param b Blue value
 * @return Combined color in the format 0xRRGGBB
 */
constexpr uint32_t COLOR(uint32_t r, uint32_t g, uint32_t b) {
	return ((r & 0xFF) << 16) + ((g & 0xFF) << 8) + (b & 0xFF);
}

int8_t hvStatusPos = 0;

void stateDisplayUpdateAndDelay(TickType_t delay) {

	/*
	extern MessageBufferHandle_t handlePodStateMessageBuffer;
	RxMessage rxMsgRaw;
	TickType_t delayStart = xTaskGetTickCount();
	*/

	/*

	 if(!OD_projectXXEnabled) {
	 leds.setAll(COLORS::BLACK);
	 }

	 do {
	 while(xMessageBufferReceive(handlePodStateMessageBuffer, &rxMsgRaw, sizeof(rxMsgRaw), 0) != 0) {
	 // Control message from state machine
	 if(checkRxMessage<messages::SensorF_TX_StatePod>(rxMsgRaw)) {
	 Message<messages::SensorF_TX_StatePod> podStateMsg{rxMsgRaw};
	 podState = podStateMsg.get<messages::SensorF_TX_StatePod::SensorF_TX_PodState>();
	 podTargetState = podStateMsg.get<messages::SensorF_TX_StatePod::SensorF_TX_PodState_Target>();
	 if(podState != lastPodState) {
	 lastStateChange = xTaskGetTickCount();
	 lastPodState = podState;
	 }
	 }
	 }


	 while(xMessageBufferReceive(handleSensorRMessageBuffer, &rxMsgRaw, sizeof(rxMsgRaw), 0) != 0) {
	 // Messages from SensorR
	 if(checkRxMessage<messages::SensorR_TX_Velocity>(rxMsgRaw)) {
	 Message<messages::SensorR_TX_Velocity> velocityMsg{rxMsgRaw};
	 podVelocity = velocityMsg.get<messages::SensorR_TX_Velocity::SensorR_Speed>();
	 } else if(checkRxMessage<messages::SensorR_TX_Position>(rxMsgRaw)) {
	 Message<messages::SensorR_TX_Position> positionMsg{rxMsgRaw};
	 podPosition = positionMsg.get<messages::SensorR_TX_Position::SensorR_Position>();
	 } else if(checkRxMessage<messages::SensorR_SDO_Resp>(rxMsgRaw)) {
	 Message<messages::SensorR_SDO_Resp> sdoRespMsg{rxMsgRaw};
	 uint16_t sdoId = sdoRespMsg.get<messages::SensorR_SDO_Resp::SensorR_SDO_ID>();
	 if(sdoId == messages::SensorR_SDO_Resp::SensorR_SDO_ID::ESTIMATEDRUNLENGTH) {
	 estimatedRunLength = sdoRespMsg.get<signals::SensorR_OD_EstimatedRunLength>();
	 } else if(sdoId == messages::SensorR_SDO_Resp::SensorR_SDO_ID::RUNPROFILE_V0) {
	 runProfileV0 = sdoRespMsg.get<signals::SensorR_OD_RunProfile_V0>();
	 } else if(sdoId == messages::SensorR_SDO_Resp::SensorR_SDO_ID::RUNPROFILE_V1) {
	 runProfileV1 = sdoRespMsg.get<signals::SensorR_OD_RunProfile_V1>();
	 } else if(sdoId == messages::SensorR_SDO_Resp::SensorR_SDO_ID::RUNPROFILE_V2) {
	 runProfileV2 = sdoRespMsg.get<signals::SensorR_OD_RunProfile_V2>();
	 } else if(sdoId == messages::SensorR_SDO_Resp::SensorR_SDO_ID::RUNPROFILE_V3) {
	 runProfileV3 = sdoRespMsg.get<signals::SensorR_OD_RunProfile_V3>();
	 } else if(sdoId == messages::SensorR_SDO_Resp::SensorR_SDO_ID::RUNPROFILE_V4) {
	 runProfileV4 = sdoRespMsg.get<signals::SensorR_OD_RunProfile_V4>();
	 }
	 }
	 }

	 // Velocity and position animation
	 if(podState == POD_STATE::POD_LAUNCHING) {
	 // Get maximum profile velocity
	 float vMax = runProfileV0;
	 if(runProfileV1 > vMax) {
	 vMax = runProfileV1;
	 }
	 if(runProfileV2 > vMax) {
	 vMax = runProfileV2;
	 }
	 if(runProfileV3 > vMax) {
	 vMax = runProfileV3;
	 }
	 if(runProfileV4 > vMax) {
	 vMax = runProfileV4;
	 }

	 float velocityPercent = 0;
	 if(vMax != 0) {
	 velocityPercent = podVelocity / vMax;
	 }
	 if(velocityPercent > 1) {
	 velocityPercent = 1;
	 }
	 if(velocityPercent < 0) {
	 velocityPercent = 0;
	 }

	 float positionPercent = 0;
	 if(estimatedRunLength != 0) {
	 positionPercent = podPosition / estimatedRunLength;
	 }
	 if(positionPercent > 1) {
	 positionPercent = 1;
	 }
	 if(positionPercent < 0) {
	 positionPercent = 0;
	 }


	 int16_t velocityLastLed = leds.RIGHT_LED * (1.0f - velocityPercent);
	 int16_t positionLastLed = leds.RIGHT_LED * (1.0f - positionPercent);

	 for(int16_t i = leds.RIGHT_LED; i >= 0; i--) {
	 if(i >= velocityLastLed && i >= positionLastLed) {
	 leds.set(i, 0x007E56);
	 leds.set(-i, 0x007E56);
	 } else if(i >= velocityLastLed){
	 leds.set(i, 0x0000FF);
	 leds.set(-i, 0x0000FF);
	 } else if(i >= positionLastLed){
	 leds.set(i, 0x00FF00);
	 leds.set(-i, 0x00FF00);
	 }
	 }
	 }



	 // Blink red when going into fault state
	 if(podState == POD_STATE::POD_FAULT) {
	 if(xTaskGetTickCount() - lastStateChange <= 2000) {
	 TickType_t passedTime = xTaskGetTickCount() - lastStateChange;
	 passedTime /= 100;
	 if(passedTime % 2 == 0) {
	 leds.setAll(COLORS::RED);
	 } else {
	 leds.setAll(COLORS::BLACK);
	 }
	 }
	 }

	 // Green light on launch or pushing
	 if(podState == POD_STATE::POD_LAUNCHING || podState == POD_STATE::POD_PUSHABLE) {
	 if(xTaskGetTickCount() - lastStateChange <= 500) {
	 leds.setAll(COLORS::GREEN);
	 }else if(xTaskGetTickCount() - lastStateChange <= 1000) {
	 float brightness = 1.0f - (xTaskGetTickCount() - lastStateChange - 500) / 1000.0f;
	 leds.setAll(COLORS::GREEN, brightness);
	 }
	 }

	 // Other state change -> just blink white once for one second
	 if(podState == POD_STATE::POD_IDLE || podState == POD_STATE::POD_LAUNCH_PREPARATION || podState == POD_STATE::POD_SAFE_TO_APPROACH) {
	 if(xTaskGetTickCount() - lastStateChange <= 1000) {
	 leds.setAll(COLORS::WHITE);
	 }
	 }

	 // HV system is precharging -> orange whipe wandering around the pod
	 if(podState == POD_STATE::POD_LAUNCH_PREPARATION && podTargetState == POD_STATE::POD_READY_TO_LAUNCH) {
	 hvStatusPos = (hvStatusPos + 1) % leds.RIGHT_LED;
	 for(int16_t j = 0; j <= 5; j++) {
	 if((hvStatusPos-j) >= 0 && (hvStatusPos-j) <= leds.RIGHT_LED) {
	 if(j == 0) {
	 leds.set((hvStatusPos-j), COLORS::BLACK);
	 leds.set(-(hvStatusPos-j), COLORS::BLACK);
	 } else if (j == 5) {
	 leds.set((hvStatusPos-j), COLORS::BLACK);
	 leds.set(-(hvStatusPos-j), COLORS::BLACK);
	 } else {
	 leds.set((hvStatusPos-j), COLORS::ORANGE);
	 leds.set(-(hvStatusPos-j), COLORS::ORANGE);
	 }
	 }
	 }
	 }

	 // HV system is on -> red whipe wandering around the pod
	 if(podState == POD_STATE::POD_READY_TO_LAUNCH || podState == POD_STATE::POD_LAUNCHING) {
	 hvStatusPos = (hvStatusPos + 1) % leds.RIGHT_LED;
	 for(int16_t j = 0; j <= 5; j++) {
	 if((hvStatusPos-j) >= 0 && (hvStatusPos-j) <= leds.RIGHT_LED) {
	 if(j == 0) {
	 leds.set((hvStatusPos-j), COLORS::BLACK);
	 leds.set(-(hvStatusPos-j), COLORS::BLACK);
	 } else if (j == 5) {
	 leds.set((hvStatusPos-j), COLORS::BLACK);
	 leds.set(-(hvStatusPos-j), COLORS::BLACK);
	 } else {
	 leds.set((hvStatusPos-j), COLORS::RED);
	 leds.set(-(hvStatusPos-j), COLORS::RED);
	 }
	 }
	 }
	 }

	leds.update();

	if (xTaskGetTickCount() - delayStart < delay) {

		TickType_t remainingTime = delay - (xTaskGetTickCount() - delayStart);

		osDelay(remainingTime);
		//break;
	}
	 */

	//while(xTaskGetTickCount() - delayStart < delay);
	//leds.update();

}

/**
 * Fade a color from black to full brightness
 * @param delayMs	Delay between updates
 * @param color		Color to use
 */
void fadeBrightness(uint32_t delayMs, uint32_t color) {
	// Fade mu-zero green from off to full brightness
	for (int16_t i = 0; i <= 255; i++) {
		leds.setAll(color, i / 255.0);

		// "Center LEDs" they mark the front of the pod for now
		leds.set(0, 127, 127, 127);

		stateDisplayUpdateAndDelay(pdMS_TO_TICKS(delayMs));
	}
}

void snake(uint32_t delayMs, uint32_t color, float brightness) {
	for (int16_t i = 0; i <= leds.RIGHT_LED - 4; i++) {
		leds.set((i), color, brightness);
		leds.set((i + 1), color, brightness / 2.0f);
		leds.set((i + 2), color, brightness / 3.0f);
		leds.set((i + 3), color, brightness / 4.0f);
		leds.set((i + 4), color, brightness / 5.0f);
		leds.set((-i), color, brightness);
		leds.set((-i - 1), color, brightness / 2.0f);
		leds.set((-i - 2), color, brightness / 3.0f);
		leds.set((-i - 3), color, brightness / 4.0f);
		leds.set((-i - 4), color, brightness / 5.0f);
		stateDisplayUpdateAndDelay(pdMS_TO_TICKS(delayMs));
	}
}

void flash(uint32_t color, uint16_t numbers) {
	for (int i = 0; i < numbers; i++) {
		for (int16_t i = 0; i <= leds.RIGHT_LED; i = i + 5) {
			leds.set((i), color, 1.0f);
			leds.set((i + 1), color, 1.0f);
			leds.set((i + 2), color, 1.0f);
			leds.set((i + 3), color, 1.0f);
			leds.set((i + 4), color, 1.0f);
			leds.set((-i), color, 1.0f);
			leds.set((-i - 1), color, 1.0f);
			leds.set((-i - 2), color, 1.0f);
			leds.set((-i - 3), color, 1.0f);
			leds.set((-i - 4), color, 1.0f);
			stateDisplayUpdateAndDelay(pdMS_TO_TICKS(10));
		}
		for (int16_t i = 0; i <= leds.RIGHT_LED; i = i + 5) {
			leds.set((i), COLORS::MU_GREEN, 0.1f);
			leds.set((i + 1), COLORS::MU_GREEN, 0.1f);
			leds.set((i + 2), COLORS::MU_GREEN, 0.1f);
			leds.set((i + 3), COLORS::MU_GREEN, 0.1f);
			leds.set((i + 4), COLORS::MU_GREEN, 0.1f);
			leds.set((-i), COLORS::MU_GREEN, 0.1f);
			leds.set((-i - 1), COLORS::MU_GREEN, 0.1f);
			leds.set((-i - 2), COLORS::MU_GREEN, 0.1f);
			leds.set((-i - 3), COLORS::MU_GREEN, 0.1f);
			leds.set((-i - 4), COLORS::MU_GREEN, 0.1f);
			stateDisplayUpdateAndDelay(pdMS_TO_TICKS(15));
		}
	}
}

/**
 * Whipe the LEDs with a color bar
 * @param delayMs		Delay between updates
 * @param whipeColor	Color for the whipe effect
 * @param outColor		Color after the whipe
 * @param outBrightness	Brightness for the color after the whipe
 */
void whipe(uint32_t delayMs, uint32_t whipeColor, uint32_t outColor,
		float outBrightness) {
	constexpr int16_t WHIPE_WIDTH = 5;
	for (int16_t i = 0; i <= leds.RIGHT_LED + WHIPE_WIDTH; i++) {
		for (int16_t j = 0; j < WHIPE_WIDTH; j++) {
			if ((i - j) >= 0 && (i - j) <= leds.RIGHT_LED) {
				leds.set((i - j), whipeColor);
				leds.set(-(i - j), whipeColor);
			}
		}
		if ((i - WHIPE_WIDTH) >= 0 && (i - WHIPE_WIDTH) <= leds.RIGHT_LED) {
			leds.set((i - WHIPE_WIDTH), outColor, outBrightness);
			leds.set(-(i - WHIPE_WIDTH), outColor, outBrightness);
		}

		stateDisplayUpdateAndDelay(pdMS_TO_TICKS(delayMs));
	}
}

void multiwhipe(uint16_t numberOfWhipes, uint32_t delayMs, uint32_t whipeColor,
		uint32_t outColor, float outBrightness) {
	constexpr int16_t WHIPE_WIDTH = 5;
	for (int16_t i = 0; i <= leds.RIGHT_LED + WHIPE_WIDTH + numberOfWhipes * 30;
			i++) {
		for (int k = 0; k < numberOfWhipes; k++) {
			for (int16_t j = 0; j < WHIPE_WIDTH; j++) {
				if ((i - (30 * k) - j) >= 0
						&& (i - (30 * k) - j) <= leds.RIGHT_LED) {
					leds.set((i - (30 * k) - j), whipeColor);
					leds.set(-(i - (30 * k) - j), whipeColor);
				}
			}
			if ((i - (30 * k) - WHIPE_WIDTH) >= 0
					&& (i - (30 * k) - WHIPE_WIDTH) <= leds.RIGHT_LED) {
				leds.set((i - (30 * k) - WHIPE_WIDTH), COLORS::BLACK,
						outBrightness);
				leds.set(-(i - (30 * k) - WHIPE_WIDTH), COLORS::BLACK,
						outBrightness);
			}
			if ((i - (30 * (numberOfWhipes - 1)) - WHIPE_WIDTH) >= 0
					&& (i - (30 * (numberOfWhipes - 1)) - WHIPE_WIDTH)
							<= leds.RIGHT_LED) {
				leds.set((i - (30 * (numberOfWhipes - 1)) - WHIPE_WIDTH),
						outColor, outBrightness);
				leds.set(-(i - (30 * (numberOfWhipes - 1)) - WHIPE_WIDTH),
						outColor, outBrightness);
			}
		}

		stateDisplayUpdateAndDelay(pdMS_TO_TICKS(delayMs));
	}
}

/**
 * Rainbow animation travelling backwards
 * @param numberOfCycles Duration
 */
void rainbowAnimation(int numberOfCycles = 400) {
	float h = 0.0f;
	hsv myHsv;
	myHsv.s = 1.0f;
	myHsv.v = 1.0f;
	int32_t counter = 0;
	for (int cnter = 0; cnter < numberOfCycles; cnter++) {
		h -= 5.0f;
		if (h < 0.0f) {
			h += 360;
		}
		for (int16_t i = 0; i <= leds.RIGHT_LED; i++) {
			myHsv.h = fmodf(h + 2 * i, 360.0f);
			rgb myRgb = hsv2rgb(myHsv);
			if (i <= counter) {
				leds.set(i, myRgb.r * 255, myRgb.g * 255, myRgb.b * 255);
				leds.set(-i, myRgb.r * 255, myRgb.g * 255, myRgb.b * 255);
			}
		}
		counter++;
		stateDisplayUpdateAndDelay(pdMS_TO_TICKS(20));
	}
}

void blinking(float frequency, int cycles) {
	leds.setAll(COLORS::MU_GREEN, 0.2f);
	for (int i = 0; i < cycles; i++) {
		if (i < cycles * 0.7) {
			leds.setAll(COLORS::MU_GREEN, 0.2f);
		} else {
			leds.setAll(COLORS::MU_PINK, 0.9f);
		}
		stateDisplayUpdateAndDelay(pdMS_TO_TICKS(50));
	}
}

void pingpong(uint32_t color, uint16_t whipeWidth, uint16_t numberOfPongs) {
	for (int i = 0; i < numberOfPongs; i++) {
		for (int i = 0; i <= leds.RIGHT_LED + whipeWidth; i = i + 1) {
			for (int j = 0; j <= whipeWidth; j++) {
				float brightness = (whipeWidth - j) / float(whipeWidth);
				if (i - j >= 0 && i - j <= leds.RIGHT_LED) {
					leds.set(i - j, color, brightness);
					leds.set(-i + j, color, brightness);
				}
			}
			if (i - whipeWidth - 1 >= 0) {
				leds.set(i - whipeWidth - 1, COLORS::BLACK, 1.0f);
				leds.set(-i + whipeWidth + 1, COLORS::BLACK, 1.0f);
			}
			stateDisplayUpdateAndDelay(pdMS_TO_TICKS(10));
		}
		for (int i = leds.RIGHT_LED; i >= -whipeWidth; i = i - 1) {
			for (int j = 0; j <= whipeWidth; j++) {
				float brightness = (whipeWidth - j) / float(whipeWidth);
				if (i + j >= 0 && i + j <= leds.RIGHT_LED) {
					leds.set(i + j, color, brightness);
					leds.set(-i - j, color, brightness);
				}
			}
			if (i + whipeWidth + 1 >= 0) {
				leds.set(i + whipeWidth + 1, COLORS::BLACK, 1.0f);
				leds.set(-i - whipeWidth - 1, COLORS::BLACK, 1.0f);
			}
			stateDisplayUpdateAndDelay(pdMS_TO_TICKS(10));

		}
	}
}

void stateOfCharge() {

}

/**
 * Animation that pulses in mu-zero green
 * @param numberOfCycles Duration
 */
void muZeroGreenPulseAnimation(int numberOfCycles = 800, int speed = 30) {
	int32_t off = 0;
	int16_t ledCounter = 0;
	leds.setAll(COLORS::MU_GREEN, 0.4);
	for (int cnter = 0; cnter < numberOfCycles; cnter++) {
		off++;
		ledCounter++;

		for (int16_t i = 0; i <= leds.RIGHT_LED; i++) {
			int32_t pos = i - off;

			if (i + 10 <= off) {
				float brightness = 0.4f
						+ 0.6f
								* powf(
										(1.0f + sinf(TWO_PI * pos / 70.0f))
												/ 2.0f, 7.0f);
				leds.set(i, COLORS::MU_GREEN, brightness);
				leds.set(-i, COLORS::MU_GREEN, brightness);
			}
		}

		stateDisplayUpdateAndDelay(pdMS_TO_TICKS(speed));
	}
}

bool pulse2(unsigned int start, unsigned int end, uint32_t color,
		uint32_t tick) {
	float brightness = 0.4f
			+ 0.6f * powf((1.0f + sinf(TWO_PI * tick / 100.0f)) / 2.0f, 7.0f);
	for (unsigned int i = start; i < end; i++) {
		leds.set(i, color, brightness);
	}
}

bool blinking2(unsigned int start, unsigned int end, uint32_t color1,
		float brightness1, uint32_t color2, float brightness2, uint32_t tick) {
	if (tick % 2 == 0) {
		for (unsigned int i = start; i < end; i++) {
			leds.set(i, color1, brightness1);
		}
	} else {
		for (unsigned int i = start; i < end; i++) {
			leds.set(i, color2, brightness2);
		}
	}
	return tick >= 2;
}

bool whipe2(int16_t start, int16_t end, uint32_t color1, float brightness1,
		uint32_t color2, float brightness2, uint32_t tick, int16_t step) {
	if (start < end) {
		int16_t whipeEnd = tick * step + start;
		for (int16_t i = start; i < std::min(whipeEnd, end); i++) {
			leds.set(i, color1, brightness1);
		}
		for (int16_t i = whipeEnd; i < end; i++) {
			leds.set(i, color2, brightness2);
		}
		return step * tick > static_cast<uint32_t>(end - start);
	} else {
		start++;
		end++;
		int16_t whipeEnd = std::max(start - tick * step,
				static_cast<uint32_t>(end));
		for (int16_t i = end; i < whipeEnd; i++) {
			leds.set(i, color2, brightness2);
		}
		for (int16_t i = whipeEnd; i < start; i++) {
			leds.set(i, color1, brightness1);
		}
		return step * tick > static_cast<uint32_t>(start - end);
	}
}

float learp(float a, float b, float alpha) {
	return a * alpha + b * (1 - alpha);
}

void whipe3(int16_t start, int16_t end, uint32_t color1, float brightness1,
		uint32_t color2, float brightness2, float blend, float time) {
	float r1 = ((color1 >> 16) & 0xFF) / 255.0f;
	float g1 = ((color1 >> 8) & 0xFF) / 255.0f;
	float b1 = (color1 & 0xFF) / 255.0f;
	float r2 = ((color2 >> 16) & 0xFF) / 255.0f;
	float g2 = ((color2 >> 8) & 0xFF) / 255.0f;
	float b2 = (color2 & 0xFF);

	int16_t width = end - start;
	for (int16_t i = 0; i < width; i++) {
		float pos = i / (float) (width - 1);
		float alpha = 1.0f / (1.0 + std::exp(-(blend * (pos - time))));
		alpha = std::max(0.0f, std::min(alpha, 1.0f));
		float r = learp(r1, r2, alpha);
		float b = learp(b1, b2, alpha);
		float g = learp(g1, g2, alpha);
		float lum = learp(brightness1, brightness2, alpha);
		leds.set(i + start, static_cast<uint32_t>(r * 255),
				static_cast<uint32_t>(g * 255), static_cast<uint32_t>(b * 255),
				lum);
	}
}

float easeOutQubic(float time) {
	return 1.0 - std::pow(1.0 - time, 3.0);
}

float easeInQubic(float time) {
	return time * time * time;
}

float easeInOutCubic(float x) {
	return x < 0.5 ? 4 * x * x * x : 1 - std::pow(-2 * x + 2, 3) / 2;

}

float easeOutExpo(float x) {
	return x == 1 ? 1 : 1 - std::pow(2, -10 * x);
}

float easeInExpo(float x) {
	return x == 0 ? 0 : std::pow(2, 10 * x - 10);
}

float clamp(float x, float min, float max) {
	return std::min(max, std::max(min, x));
}

std::function<float(float)> step(float step) {
	return [step](float x) {
		if (x >= step)
			return 1.0f;
		else
			return 0.0f;
	};
}

struct color_t {
	uint32_t m_hexColor;
	explicit color_t() :
			m_hexColor(0) {
	}
	explicit color_t(uint32_t rgb, float lum) :
			m_hexColor(
					(rgb & 0xFFFFFF) | (static_cast<uint32_t>(lum * 255) << 24)) {
	}
	explicit color_t(float r, float g, float b, float lum) :
			m_hexColor(
					(static_cast<uint32_t>(clamp(r, 0.0f, 1.0f) * 255) << 16)
							| (static_cast<uint32_t>(clamp(g, 0.0f, 1.0f) * 255)
									<< 8)
							| (static_cast<uint32_t>(clamp(b, 0.0f, 1.0f) * 255))
							| (static_cast<uint32_t>(clamp(lum, 0.0f, 1.0f)
									* 255) << 24)) {

	}
	uint32_t uint_r() const {
		return (m_hexColor >> 16) & 0xFF;
	}
	uint32_t uint_g() const {
		return (m_hexColor >> 8) & 0xFF;
	}
	uint32_t uint_b() const {
		return m_hexColor & 0xFF;
	}
	uint32_t uint_lum() const {
		return (m_hexColor >> 24) & 0xFF;
	}
	float r() const {
		return uint_r() / 255.0f;
	}
	float g() const {
		return uint_g() / 255.0f;
	}
	float b() const {
		return uint_b() / 255.0f;
	}
	float lum() const {
		return uint_lum() / 255.0f;
	}
};

color_t operator+(const color_t &c1, const color_t &c2) {
	return color_t(c1.r() + c2.r(), c1.g() + c2.g(), c1.b() + c2.b(),
			c1.lum() + c2.lum());
}

color_t operator*(const color_t &c1, const color_t &c2) {
	return color_t(c1.r() * c2.r(), c1.g() * c2.g(), c1.b() * c2.b(),
			c1.lum() * c2.lum());
}

color_t operator*(const float &scalar, const color_t &color) {
	return color_t(scalar * color.r(), scalar * color.g(), scalar * color.b(),
			scalar * color.lum());
}

color_t operator*(const color_t &color, const float &scalar) {
	return color_t(scalar * color.r(), scalar * color.g(), scalar * color.b(),
			scalar * color.lum());
}

color_t linmix(const color_t &c1, const color_t &c2, float alpha) {
	return (alpha * c1) + (1 - alpha) * c2;
}

float identity(float x) {
	return x;
}

float identity1(float x, float y) {
	return x;
}
float identity2(float x, float y) {
	return y;
}

void animate(int16_t start, int16_t end, int16_t step, color_t c1, color_t c2,
		std::function<float(float)> activationFunction,
		std::function<float(float)> timeFunction,
		std::function<float(float pos, float time)> animationFunction,
		std::function<color_t(const color_t&, const color_t&, float)> mixFunction,
		float time) {
	if (start == end)
		return;

	int16_t ledCount = std::abs(start - end);
	int16_t dir = end > start ? 1 : -1;
	int16_t stepCount = (ledCount + step - 1) / step; //integer ceil division.

	time = timeFunction(time);
	time = clamp(time, 0.0f, 1.0f);

	for (int16_t i = 0; i < stepCount; i++) {
		float pos = (step * i + step / 2) / (float) (ledCount - 1);
		float animatedAlpha = animationFunction(pos, time);
		float alpha = activationFunction(animatedAlpha);
		color_t color = mixFunction(c1, c2, alpha);
		for (int16_t j = 0; j < step && j < end; j++) {
			leds.set(start + dir * i * step + j, color.uint_r(), color.uint_g(),
					color.uint_b(), color.lum());
		}
	}
}

float absAnimation(float pos, float time) {
	pos = 2.0f * pos - 1.0f;
	return std::abs(pos);
}

std::function<float(float pos, float time)> shiftAnimation(
		std::function<float(float pos, float time)> animation, float scalar =
				1.0f, bool wrap = true) {
	return [animation, scalar, wrap](float pos, float time) {
		float newPos = scalar * time + pos;
		if (wrap)
			newPos = std::fmod(newPos, 1.0f);
		return animation(newPos, time);
	};
}

std::function<float(float pos, float time)> repeatAnimation(
		std::function<float(float pos, float time)> animation, float scalar) {
	return [animation, scalar](float pos, float time) {
		float newPos = scalar * pos;
		newPos = std::fmod(newPos, 1.0f);
		return animation(newPos, time);
	};
}

std::function<float(float pos, float time)> offsetAnimation(
		std::function<float(float pos, float time)> animation, float offset,
		bool wrap = true) {
	return [animation, offset, wrap](float pos, float time) {
		float newPos = pos + offset;
		if (wrap)
			newPos = std::fmod(newPos, wrap);
		return animation(newPos, time);
	};
}

float sigmoid(float x) {
	x = 2 * x - 1;
	x = x * 4;
	return 1.0f / (1.0f + std::exp(-x));
}

float sigmoid_bumerang(float x) {
	if (x < 0.5f) {
		return sigmoid(2 * x) * 0.5;
	} else {
		return 0.5 + sigmoid(2 * (x - 0.5)) * 0.5;
	}
}

float hyperbolic_tangent(float x) {
	x = (x - 0.5f) * 2.0f;
	x = x * 4;
	return (std::tanh(x) + 1.0f) / 2.0f;
}

float gaussian(float x) {
	return std::exp(-std::pow(4 * x - 2, 2));
}

std::function<float(float)> gaussian_curve(float steepness) {
	return [steepness](float x) {
		return std::exp(-std::pow(steepness * (4 * x - 2), 2));
	};
}

float sinus(float x) {
	x = 2.0f * PI * x;
	float sin = std::sin(x);
	return (sin + 1.0f) / 2.0f;
}

float sinus1(float x, float y) {
	return sinus(x);
}

float sinus2(float x, float y) {
	return sinus(y);
}

float absSinus2(float x, float y) {
	return std::abs(std::sin(2 * PI * y));
}

float randomForier(float x, float y) {
	return (sin(20 * x) + cos(8 * x + 0.1) + sin(28 * x + 3) + cos(6 * x + 2))
			* 0.125 + 0.5;
}

std::function<float(float, float)> bumerang(
		std::function<float(float, float)> animation) {
	return [animation](float pos, float time) {
		time = 2 * time;
		if (time > 1) {
			time = 2.0 - time;
		}
		return animation(pos, time);
	};
}

void blinkingAnimation(int16_t start, int16_t end, int16_t step_,
		const color_t &c1, const color_t &c2, float time) {
	animate(start, end, step_, c1, c2, step(0.5), identity, identity2, linmix,
			time);
}

void whipeEaseOutAnimation(int16_t start, int16_t end, int16_t step,
		const color_t &c1, const color_t &c2, float time) {
	animate(start, end, step, c1, c2, identity, easeOutQubic,
			shiftAnimation(offsetAnimation(identity1, -1.0f, false), 2.0f,
					false), linmix, time);
}

void waveAnimation(int16_t start, int16_t end, int16_t step, const color_t &c1,
		const color_t &c2, float time) {
	animate(start, end, step, c1, c2, easeInQubic, identity,
			shiftAnimation(sinus1, 1.0f, true), linmix, time);
}

void pulseAnimation(int16_t start, int16_t end, int16_t step, const color_t &c1,
		const color_t &c2, float time) {
	animate(start, end, step, c1, c2, easeInQubic, identity, absSinus2, linmix,
			time);
}

void pingPongAnimation(int16_t start, int16_t end, int16_t step,
		const color_t &c1, const color_t &c2, float time) {
	float steepness = 2.0f;
	animate(start, end, step, c1, c2, gaussian_curve(steepness), identity,
			bumerang(
					shiftAnimation(offsetAnimation(identity1, -0.5f, false),
							1.0f, false)), linmix, time);
}

void smoothPingPongAnimation(int16_t start, int16_t end, int16_t step,
		const color_t &c1, const color_t &c2, float time) {
	float steepness = 2.0f;
	animate(start, end, step, c1, c2, gaussian_curve(steepness),
			sigmoid_bumerang,
			bumerang(
					shiftAnimation(offsetAnimation(identity1, -0.5f, false),
							1.0f, false)), linmix, time);
}

/**
 * Main task for Project XX
 * @param pvParams unused
 */
static void projectXXFunction(void *pvParams) {
	osDelay(100);

// Make sure that while DCDC powers up, all LEDs are black
	for (int i = 0; i < 20; i++) {
		leds.setAll(COLORS::BLACK);
		leds.update();
		osDelay(pdMS_TO_TICKS(50));
	}

	leds.setAll(COLORS::MU_GREEN, 1.0f);
	leds.update();
	while(true){
		leds.update();
		osDelay(pdMS_TO_TICKS(50));
	}

	// startup animation.
	/*
	{
		uint32_t startupTicks = 200;
		for (uint32_t tick = 0; tick < startupTicks; tick++) {
			float time = tick / (float) startupTicks;
			whipeEaseOutAnimation(1, 151, 1, color_t(COLORS::GREEN, 0.5f), color_t(COLORS::BLACK, 0.0f), time);
			whipeEaseOutAnimation(NUM_LEDS, 150, 1, color_t(COLORS::RED, 0.5f), color_t(COLORS::BLACK, 0.0f), time);
			leds.update();
			osDelay(pdMS_TO_TICKS(50));
		}
	}

	uint32_t tick = 0;
	uint32_t duration = 500;
	while (true) {
		float time = std::fmod(tick / (float) duration + 0.25f, 1.0f);
		pulseAnimation(1, 66, 1, color_t(COLORS::RED, 0.5f), color_t(COLORS::RED, 0.3f), time);
		//whipeEaseOutAnimation(1, 61, 1, color_t(COLORS::MU_GREEN, 1.0f), color_t(COLORS::BLACK, 0.0f), time);
		//blinkingAnimation(1, 37, 1, color_t(COLORS::MU_GREEN, 1.0f), color_t(COLORS::BLACK, 0.0f), time);

		leds.update();

		if (tick == duration) {
			tick = 0;
		} else {
			tick++;
		}
		osDelay(pdMS_TO_TICKS(50));
	}

	/*
	 uint32_t tick = 0;
	 while (true) {
	 float time = tick / 200.0f;
	 whipe3(11, 21, COLORS::RED, 1.0f, COLORS::GREEN, 0.1f, 5.0f,
	 easeOutQubic(time));
	 tick++;
	 if (tick >= 200) {
	 tick = 0;
	 }
	 leds.update();
	 osDelay(pdMS_TO_TICKS(10));
	 }
	 /*
	 uint32_t tick = 0;
	 while (true) {
	 //volatile bool reset1 = whipe2(11, 21, COLORS::RED, 0.1f, COLORS::GREEN, 0.1f, tick, 2);
	 volatile bool reset2 = whipe2(20, 10, COLORS::RED, 0.1f, COLORS::GREEN, 0.1f, tick, 3);

	 if(reset2){
	 tick = 0;
	 } else {
	 tick++;
	 }
	 leds.update();
	 osDelay(pdMS_TO_TICKS(500));
	 }
	 */

// Start up animation
	leds.setAll(COLORS::BLACK);
	snake(35, COLORS::MU_GREEN, 0.3f);

//possible functions to add: fast flash with fading tail wandering around the pod, bouncing flashes with fading tails, state of charge

//pingpong(COLORS::BRIGHT_PURPLE, 20, 5);
//muZeroGreenPulseAnimation(300, 20);

	while (1) {
		//leds.setAll(COLORS::WHITE, 1.0f);

		whipe(10, COLORS::MU_RED, COLORS::MU_GREEN, 0.2f);
		whipe(10, COLORS::MU_RED, COLORS::MU_PINK, 0.9f);
		whipe(10, COLORS::MU_RED, COLORS::MU_GREEN, 0.2f);
		muZeroGreenPulseAnimation(800, 20);

		whipe(10, COLORS::MU_PINK, COLORS::MU_GREEN, 0.2f);
		whipe(10, COLORS::MU_PINK, COLORS::MU_PINK, 0.9f);
		whipe(10, COLORS::MU_PINK, COLORS::MU_GREEN, 0.2f);
		muZeroGreenPulseAnimation(800, 20);

		whipe(10, COLORS::MU_PINK, COLORS::MU_GREEN, 0.2f);
		whipe(10, COLORS::MU_PINK, COLORS::MU_PINK, 0.9f);
		whipe(10, COLORS::MU_PINK, COLORS::MU_GREEN, 0.2f);
		rainbowAnimation();
		//leds.update();
		//osDelay(10);
	}
}

#endif /* INCLUDE_PROJECTXX_HPP_ */
