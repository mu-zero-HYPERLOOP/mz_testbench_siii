/*
 * Sensor.h
 *
 *  Created on: 21.10.2020
 *      Author: Felix
 */

#ifndef INCLUDE_SENSOR_HPP_
#define INCLUDE_SENSOR_HPP_

#include "gpio.h"

class Sensor {
public:
	Sensor();
	virtual ~Sensor();
	void init(bool, ADC_HandleTypeDef*);
	GPIO_PinState readGPIO(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
private:

};

#endif /* INCLUDE_SENSOR_HPP_ */
