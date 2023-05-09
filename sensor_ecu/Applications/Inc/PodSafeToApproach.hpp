/*
 * PodSafeToApproach.h
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef INC_PODSAFETOAPPROACH_HPP_
#define INC_PODSAFETOAPPROACH_HPP_

#include <State.hpp>

class PodSafeToApproach: public State {
public:
	PodSafeToApproach() : State(POD_SAFE_TO_APPROACH) {}

	void setup() override;

	void update() override;

	void dispose() override;

private:
};

#endif /* INC_PODSAFETOAPPROACH_HPP_ */
