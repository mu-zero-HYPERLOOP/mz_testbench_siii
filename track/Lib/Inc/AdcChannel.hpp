/*
 * AdcChannel.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include <cinttypes>
#include "FreeRTOS.h"
#include "cmsis_os.h"

class AdcChannel {
public:
	explicit AdcChannel() {

	}

	uint16_t get() {
		return m_value;
	}

	void setValue(uint16_t value){
		m_value = value;
	}

private:
	uint16_t m_value;
};
