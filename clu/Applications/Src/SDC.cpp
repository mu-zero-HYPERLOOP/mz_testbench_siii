/*
 * SDC.cpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */
#include "SDC.hpp"

SDC::SDC(const SDCConfig& config, SDCStatus initalStatus) : m_gpio(config.m_gpio.m_port, config.m_gpio.m_pin), m_status(initalStatus) {
	open();
}

void SDC::open(){
	m_gpio.reset();
	m_status = SDC_OPEN;
}

void SDC::close(){
	m_gpio.set();
	m_status = SDC_CLOSED;
}

SDC::SDCStatus SDC::status(){
	return m_status;
}



