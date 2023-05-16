/*
 * Sensor.cpp
 *
 *  Created on: 21.10.2020
 *      Author: Felix
 */

#include <Sensor.hpp>
#include "adc.h"
#include "log.h"


Sensor::Sensor(){

}

/*
 * if an analog sensor is used, the adc object must be given
 * if a digital sensor is used adc parameter should be NULL
 */
void Sensor::init(bool analog, ADC_HandleTypeDef* hadc) {
	if(analog)
	{
			HAL_ADC_Start_IT(hadc);
			HAL_ADC_Start(hadc);
	}

}
GPIO_PinState Sensor::readGPIO(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
	return HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
}

Sensor::~Sensor() {
	// TODO Auto-generated destructor stub
}

