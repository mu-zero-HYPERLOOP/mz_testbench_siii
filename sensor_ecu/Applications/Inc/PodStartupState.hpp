/*
 * PodStartupState.hpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef INC_PODSTARTUPSTATE_HPP_
#define INC_PODSTARTUPSTATE_HPP_

#include <State.hpp>
#include "estdio.hpp"

class PodStartupState: public State {
public:
	PodStartupState() : State(POD_OFF) {}

	void setup() override;

	void update() override;

	void dispose() override;
private:
};

#endif /* INC_PODSTARTUPSTATE_HPP_ */