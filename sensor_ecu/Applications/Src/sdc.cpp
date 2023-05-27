/*
 * sdc.hpp
 *
 *  Created on: May 21, 2023
 *      Author: OfficeLaptop
 */

#include "sdc.hpp"
#include "GPIOWriteController.hpp"
#include "gpio.h"

namespace sdc {

GPIOWriteController m_sdcGpio(SDC_GPIO_Port, SDC_Pin);

void open(){
	m_sdcGpio.reset();
}

void close(){
	m_sdcGpio.set();
}


void init(){
	close();
}

void update(){
}

}
