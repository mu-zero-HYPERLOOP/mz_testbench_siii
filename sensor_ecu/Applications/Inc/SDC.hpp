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
	static SDC& getInstance() {
		static SDC instance {g_peripherals.m_sdcConfig, SDC_OPEN};
		return instance;
	}

	enum SDCStatus : bool {
		SDC_OPEN = true, SDC_CLOSED = false
	};
public:
	SDC(SDC&) = delete;
	SDC(SDC&&) = delete;
	SDC& operator=(SDC&) = delete;
	SDC& operator=(SDC&&) = delete;

	void open();
	void close();
	SDCStatus status();
private:
	explicit SDC(const SDCConfig &config, SDCStatus initalStatus = SDC_OPEN);
	GPIOWriteController m_gpio;
	SDCStatus m_status;
};
