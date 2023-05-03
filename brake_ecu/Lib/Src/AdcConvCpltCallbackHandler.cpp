/*
 * AdcConvCpltCallbackHandler.cpp
 *
 *  Created on: Apr 18, 2023
 *      Author: OfficeLaptop
 */

#include "AdcConvCpltCallbackHandler.hpp"

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	AdcConvCpltCallbackHandler::getInstance().notify(hadc);
}
