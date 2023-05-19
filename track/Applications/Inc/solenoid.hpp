/*
 * solenoid.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#pragma once

namespace solenoid {

enum PistonStatus {
	Ready = 0,
	Extended = 1
};

void init();

bool push();

void update();

}
