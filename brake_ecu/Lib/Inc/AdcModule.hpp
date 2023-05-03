/*
 * AdcModule.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "adc.h"

enum AdcModule : int {
	ADC_MODULE1, ADC_MODULE2,
};

static inline ADC_HandleTypeDef* AdcModuleToHandle(AdcModule module) {
	switch (module) {
	case ADC_MODULE1:
		return &hadc1;
	case ADC_MODULE2:
		return &hadc2;
	default:
		Error_Handler();
		return nullptr;
	}
}
