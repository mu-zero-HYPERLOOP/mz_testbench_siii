/*
 * led.c
 *
 *  Created on: Nov 17, 2020
 *      Author: Felix
 */


#include "main.h"

extern TIM_HandleTypeDef htim4;

void LED_RGB_Init() {
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);	// Green LED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);	// Orange LED
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);	// Red LED
}
void LED_RGB_Write(uint8_t green, uint8_t orange, uint8_t red) {
	htim4.Instance->CCR1 = 255 - green;
	htim4.Instance->CCR2 = 255 - orange;
	htim4.Instance->CCR3 = 255 - red;
}

void LED_Green_Write(uint8_t green) {
	htim4.Instance->CCR1 = 255 - green;
}
void LED_Orange_Write(uint8_t orange) {
	htim4.Instance->CCR2 = 255 - orange;
}
void LED_Red_Write(uint8_t red) {
	htim4.Instance->CCR3 = 255 - red;
}
