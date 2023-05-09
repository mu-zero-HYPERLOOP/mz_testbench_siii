/*
 * PodReadyToLaunchState.h
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef INC_PODREADYTOLAUNCHSTATE_HPP_
#define INC_PODREADYTOLAUNCHSTATE_HPP_

#include "canzero.hpp"
#include "NTCSensor.hpp"
#include "State.hpp"


class PodReadyToLaunchState : public State{
public:
	PodReadyToLaunchState() : State(POD_READY_TO_LAUNCH) {}


	void setup() override;

	void update() override;

	void dispose() override;
};

#endif /* INC_PODREADYTOLAUNCHSTATE_HPP_ */
