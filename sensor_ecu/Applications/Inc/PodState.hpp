/*
 * PodState.hpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include <cinttypes>

enum PodState : uint8_t{
            POD_OFF = 0,
            POD_IDLE ,
            POD_LAUNCH_PREPARATION ,
            POD_READY_TO_LAUNCH,
			POD_START_LEVITATION,
			POD_LEVITATION,
            POD_LAUNCHING,
			POD_STOP_LEVITATION,
			POD_BREAK,
			POD_END_OF_RUN,
            POD_SAFE_TO_APPROACH,
			POD_EMERGENCY,
};
