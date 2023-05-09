/*
 * PodLaunchingState.h
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef INC_PODLAUNCHINGSTATE_HPP_
#define INC_PODLAUNCHINGSTATE_HPP_

#include "State.hpp"

class PodLaunchingState : public State{
public:
	PodLaunchingState() : State(POD_LAUNCHING) {}

	void setup() override;

	void update() override;

	void dispose() override;
private:
};

#endif /* INC_PODLAUNCHINGSTATE_HPP_ */
