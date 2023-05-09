/*
 * PodBreakState.hpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#pragma once


#include "State.hpp"

class PodBreakState : public State {
public:
	PodBreakState() : State(POD_BREAK) {}

	void setup() override;

	void update() override;

	void dispose() override;
};

