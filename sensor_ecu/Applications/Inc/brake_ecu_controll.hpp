/*
 * brake_ecu_control.hpp
 *
 *  Created on: May 14, 2023
 *      Author: OfficeLaptop
 */

#pragma once

namespace brake{

namespace internal {
extern float pressureTank;
}

void init();

void update();

}
