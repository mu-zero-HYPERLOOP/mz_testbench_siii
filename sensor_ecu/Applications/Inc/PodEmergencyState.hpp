/*
 * PodEmergencyState.h
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef CANZERO_STATIC_INC_PODEMERGENCYSTATE_HPP_
#define CANZERO_STATIC_INC_PODEMERGENCYSTATE_HPP_

#include "State.hpp"

#include "estdio.hpp"

class PodEmergencyState : public State {
public:
	PodEmergencyState() : State(POD_EMERGENCY) {}

	void setup() override;

	void update() override;

	void dispose() override;
};

#endif /* CANZERO_STATIC_INC_PODEMERGENCYSTATE_HPP_ */
