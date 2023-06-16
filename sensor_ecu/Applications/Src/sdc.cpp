/*
 * sdc.hpp
 *
 *  Created on: May 21, 2023
 *      Author: OfficeLaptop
 */

#include "sdc.hpp"
#include "GPIOWriteController.hpp"
#include "gpio.h"
#include "estdio.hpp"

namespace sdc {

static GPIOWriteController m_sdcGpio(SDC_GPIO_Port, SDC_Pin);
static bool current;

void open(){
	close();
	return;
	m_sdcGpio.reset();
	if(current){
		printf("open sdc\n");
		current = false;
	}
}

void close(){
	m_sdcGpio.set();
	if(not current){
		printf("close sdc\n");
		current = true;
	}
}


void init(){
	current = true;
	open();
}

void update(){
}


}
