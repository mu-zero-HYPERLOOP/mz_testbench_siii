/*
 * SDC.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "gpio.h"
#include "peripheral_config.hpp"
#include "GPIOWriteController.hpp"


class SDC {
public:
	enum SDCStatus : bool{
		SDC_OPEN = true,
		SDC_CLOSED = false
	};
public:
	explicit SDC(const SDCConfig& config, SDCStatus initalStatus = SDC_OPEN);
	void open();
	void close();
	SDCStatus status();
private:
	GPIOWriteController m_gpio;
	SDCStatus m_status;
};
