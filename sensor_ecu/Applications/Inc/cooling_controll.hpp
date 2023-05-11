/*
 * CoolingController.hpp
 *
 *  Created on: 11 May 2023
 *      Author: karl
 */

#pragma once

namespace cooling {


enum class MODE {
	ON,
	DYNAMIC,
	OFF,
};

void setMode(MODE mode);

void update();

}
