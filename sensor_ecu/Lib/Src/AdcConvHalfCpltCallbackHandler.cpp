/*
 * AdcConvHalfCpltCallbackHandler.cpp
 *
 *  Created on: Apr 18, 2023
 *      Author: OfficeLaptop
 */


#include "AdcConvHalfCpltCallbackHandler.hpp"

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	AdcConvHalfCpltCallbackHandler::getInstance().notify(hadc);
}


