/*
 * PodLevitatingState.hpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "State.hpp"
#include "PodState.hpp"

class PodLevitationState: public State {
public:
	PodLevitationState() : State(POD_LEVITATION) {}

	void setup() override;

	void update() override;

	void dispose() override;
private:
};
