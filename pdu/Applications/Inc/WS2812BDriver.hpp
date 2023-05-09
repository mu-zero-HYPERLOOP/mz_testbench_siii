/*
 * WS2812BDriver.hpp
 *
 *  Created on: May 5, 2021
 *      Author: Flo Keck
 *
 * Driver for WS2812B LEDs
 *
 * There are some clever WS2812B drivers around (e.g. http://stm32f4-discovery.net/2018/06/tutorial-control-ws2812b-leds-stm32/ )
 * But they were too much effort to implement, so I am just using a timer in PWM mode with DMA that shifts out the whole data.
 * This needs lots of RAM, but not a problem since the PDU is not doing much at all and has plenty of ressources left.
 * We do not send the reset flag at the end explicitly, but relay on the user to call the update() function not more frequent than
 * every 10ms.
 *
 * We currently use Pin PC8, this is Timer3 Channel3
 * In CubeMX:
 * - Enable the timer
 * - Set the corresponding channel to PWM Generation
 * - Set the Counter Period to a value that get 800kHz Ouput frequency
 *   Our timer runs at 84 MHz -> Divider of 105 needed -> Set Counter period to 105 - 1 = 104
 * - Enable auto-reload preload
 * - Add the DMA Stream for the channel
 *   - Configure direction: Memory To Peripheral
 *   - Configure Increment Address: Memory
 *   - Configure Data Width: Half Word for both
 *
 * Sending data for 300 LEDs takes 9ms -> maximum update frequency is 100 Hz
 *
 */

#ifndef INCLUDE_WS2812BDRIVER_HPP_
#define INCLUDE_WS2812BDRIVER_HPP_

#include "main.h"

// Enable brightness correction to mimic the vision of the human eye
#define ENABLE_QUADRATIC_BRIGHTNESS_CORRECTION

// Enable index shift, so that center LED of the strip has index 0. Then NUM_LEDS needs to be uneven
#define ENABLE_INDEX_SHIFT_TO_CENTER
//Enable to change the rear and front position of pod:
#define SWITCH_FRONT_AND_REAR


template<int16_t NUM_LEDS>
class WS2812BDriver {
private:

	// Buffer to hold the LED data. 24 pules per LED and one final stop pulse
	static constexpr int16_t BUFFER_SIZE = NUM_LEDS * 24 + 1;
	uint16_t ledBuffer[BUFFER_SIZE];

	bool m_isRunning = false;
	TIM_HandleTypeDef* m_timerHandlePointer = NULL;
	uint32_t m_timerChannel = 0;

public:
#ifdef ENABLE_INDEX_SHIFT_TO_CENTER
	static constexpr int16_t INDEX_SHIFT = (NUM_LEDS - 1) / 2;
	static constexpr int16_t LEFT_LED = -INDEX_SHIFT;
	static constexpr int16_t RIGHT_LED = INDEX_SHIFT;
#endif

	/**
	 * Constructor for WS2812B driver
	 * @param timerHandle	Handle to the timer that does the PWM data output
	 * @param timerChannel	Channel of the timmer, e.g. TIM_CHANNEL_3
	 */
	WS2812BDriver(TIM_HandleTypeDef *timerHandle, uint32_t timerChannel) :
		m_timerHandlePointer { timerHandle }, m_timerChannel { timerChannel } {

#ifdef ENABLE_INDEX_SHIFT_TO_CENTER
			if(NUM_LEDS % 2 == 0) {
				// Number of LEDs needs to be uneven for index shift!
				while(1);
			}
#endif

		}

		virtual ~WS2812BDriver(){}

		/**
		 * Set the color of a single LED using R, G and B values
		 * @param index			Position of the LED
		 * @param r				Red value, range 0 - 255
		 * @param g				Green value, range 0 - 255
		 * @param b				Blue value, range 0 - 255
		 * @param brightness	Brightness, range 0.0 - 1.0
		 */
		void set(int16_t index, uint32_t r, uint32_t g, uint32_t b, float brightness = 1.0f) {

#ifdef SWITCH_FRONT_AND_REAR
			if(index < 0) {
				index += INDEX_SHIFT;
				index = index * (-1);
			}
			if(index > 0) {
				index -= INDEX_SHIFT;
				index = index * (-1);
			}
#endif

#ifdef ENABLE_INDEX_SHIFT_TO_CENTER
			index += INDEX_SHIFT;
#endif
			if(index < 0) {
				return;
			}
			if(index >= NUM_LEDS) {
				return;
			}
			r *= brightness;
			g *= brightness;
			b *= brightness;
#ifdef ENABLE_QUADRATIC_BRIGHTNESS_CORRECTION
			r = r * r / 255;
			g = g * g / 255;
			b = b * b / 255;
#endif
			int16_t offset = index * 24;
			uint32_t arr = m_timerHandlePointer->Instance->ARR;

			// The dataformat for the LEDs needs to be GRB
			for(int16_t i = 0; i < 8; i++) {
				// If the bit is set, the duty cylce is 66%
				// If the bit is cleared, the duty cycle is 33%
				ledBuffer[offset +  0 + i] = (g & (1 << (7 - i))) ? (2 * arr / 3) : (arr / 3);
				ledBuffer[offset +  8 + i] = (r & (1 << (7 - i))) ? (2 * arr / 3) : (arr / 3);
				ledBuffer[offset + 16 + i] = (b & (1 << (7 - i))) ? (2 * arr / 3) : (arr / 3);
			}
		}

		/**
		 * Set the color of a single LED using a RGB value
		 * @param index			Position of the LED
		 * @param rgb			Hex color code in the format 0xRRGGBB
		 * @param brightness	Brightness, range 0.0 - 1.0
		 */
		void set(int16_t index, uint32_t rgb, float brightness = 1.0f) {
			set(index, (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF, brightness);
		}

#ifdef ENABLE_INDEX_SHIFT_TO_CENTER
		/**
		 * Set all leds to a color using R, G and B values
		 * @param r				Red value, range 0 - 255
		 * @param g				Green value, range 0 - 255
		 * @param b				Blue value, range 0 - 255
		 * @param brightness	Brightness, range 0.0 - 1.0
		 */
		void setAll(uint8_t r, uint8_t g, uint8_t b, float brightness = 1.0f) {
			for(int16_t i = LEFT_LED; i <= RIGHT_LED; i++) {
				set(i, r, g, b, brightness);
			}
		}
		/**
		 * Set the color of a single LED using a RGB value
		 * @param rgb			Hex color code in the format 0xRRGGBB
		 * @param brightness	Brightness, range 0.0 - 1.0
		 */
		void setAll(uint32_t rgb, float brightness = 1.0f) {
			for(int16_t i = LEFT_LED; i <= RIGHT_LED; i++) {
				set(i, rgb, brightness);
			}
		}
#else
		/**
		 * Set all leds to a color using R, G and B values
		 * @param r				Red value, range 0 - 255
		 * @param g				Green value, range 0 - 255
		 * @param b				Blue value, range 0 - 255
		 * @param brightness	Brightness, range 0.0 - 1.0
		 */
		void setAll(uint8_t r, uint8_t g, uint8_t b, float brightness = 1.0f) {
			for(int16_t i = 0; i < NUM_LEDS; i++) {
				set(i, r, g, b, brightness);
			}
		}
		/**
		 * Set the color of a single LED using a RGB value
		 * @param rgb			Hex color code in the format 0xRRGGBB
		 * @param brightness	Brightness, range 0.0 - 1.0
		 */
		void setAll(uint32_t rgb, float brightness = 1.0f) {
			for(int16_t i = 0; i < NUM_LEDS; i++) {
				set(i, rgb, brightness);
			}
		}
#endif

		/**
		 * Update LEDs
		 */
		void update() {
			// Without calling the stop function before, we sometimes get glitches in the data
			if(m_isRunning) {
				HAL_TIM_PWM_Stop_DMA(m_timerHandlePointer, m_timerChannel);
			}
			m_isRunning = true;

			// Last entry needs to be zero, otherwise timer continues sending data. This is also our stop flag
			ledBuffer[BUFFER_SIZE - 1] = 0;

			// Send out LED data with PWM and DMA
			HAL_TIM_PWM_Start_DMA(m_timerHandlePointer, m_timerChannel, (uint32_t*)ledBuffer, BUFFER_SIZE);
		}
};



#endif /* INCLUDE_WS2812BDRIVER_HPP_ */
