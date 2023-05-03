/*
 * PodRunStopState.h
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef INC_PODRUNSTOPSTATE_HPP_
#define INC_PODRUNSTOPSTATE_HPP_

#include <State.hpp>

class PodRunStopState: public State {
public:
	PodRunStopState();

	void setup() override;

	void update() override;

	void dispose() override;

private:
};

#endif /* INC_PODRUNSTOPSTATE_HPP_ */
