/*
 * PodIdleState.h
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef INC_PODIDLESTATE_HPP_
#define INC_PODIDLESTATE_HPP_

#include "State.hpp"
#include "canzero.hpp"

#include "estdio.hpp"

class PodIdleState : public State{
public:
	PodIdleState() : State(POD_IDLE) {}

	void setup() override;

	void update() override;

	void dispose() override;
private:
};

#endif /* INC_PODIDLESTATE_HPP_ */
