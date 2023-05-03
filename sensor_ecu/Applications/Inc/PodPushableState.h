/*
 * PodPushableState.h
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef INC_PODPUSHABLESTATE_H_
#define INC_PODPUSHABLESTATE_H_

#include <State.hpp>

class PodPushableState: public State {
public:
	PodPushableState();

	void setup() override;

	void update() override;

	void dispose() override;
private:
};

#endif /* INC_PODPUSHABLESTATE_H_ */
