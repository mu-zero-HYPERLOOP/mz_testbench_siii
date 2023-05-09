/*
 * PodRunStopState.h
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef INC_PODSTOPLEVITATIONSTATE_HPP_
#define INC_PODSTOPLEVITATIONSTATE_HPP_

#include <State.hpp>

class PodStopLevitationState: public State {
public:
	PodStopLevitationState() : State(POD_STOP_LEVITATION) {}

	void setup() override;

	void update() override;

	void dispose() override;

private:
};

#endif /* INC_PODSTOPLEVITATIONSTATE_HPP_ */
