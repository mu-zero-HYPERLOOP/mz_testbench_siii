/*
 * GPIOExtiCallbackHandler.cpp
 *
 *  Created on: Apr 25, 2023
 *      Author: OfficeLaptop
 */
#include "GPIOExtiCallbackHandler.hpp"




void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	GPIOExtiCallbackHandler::getInstance().notify(pin);
}
