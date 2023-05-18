/*
 * DistanceSensor.hpp
 *
 *  Created on: May 18, 2023
 *      Author: karlsassie
 */

#pragma once

#include "AdcModule.hpp"
#include "AdcChannelController.hpp"
#include <cinttypes>

class DistanceSensor {
public:
	explicit DistanceSensor(AdcModule module, uint16_t rank) :
			m_channelController(module, rank) {

	}

	float get(bool force = false) {
		uint16_t avalue = m_channelController.get();

		//TODO write convertion to distance.

		return 0;
	}

private:
	AdcChannelController m_channelController;
};
