/*
 * PodLaunchPreparationState.h
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef INC_PODLAUNCHPREPARATIONSTATE_HPP_
#define INC_PODLAUNCHPREPARATIONSTATE_HPP_

#include "canzero.hpp"
#include "AdcChannelController.hpp"
#include "NTCSensor.hpp"
#include "ImuMaster.hpp"
#include "State.hpp"
#include "SDC.hpp"

class PodLaunchPreparationState : public State{
public:
	PodLaunchPreparationState() : State(POD_LAUNCH_PREPARATION){}

	void setup() override;

	void update() override;

	void dispose() override;
};

#endif /* INC_PODLAUNCHPREPARATIONSTATE_HPP_ */
