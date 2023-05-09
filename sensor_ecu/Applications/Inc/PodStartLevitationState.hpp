/*
 * PodStartLevitationState.hpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "State.hpp"
#include "PodState.hpp"


class PodStartLevitation: public State {
public:
	PodStartLevitation() : State(POD_SAFE_TO_APPROACH) {}

	void setup() override;

	void update() override;

	void dispose() override;

private:
};
