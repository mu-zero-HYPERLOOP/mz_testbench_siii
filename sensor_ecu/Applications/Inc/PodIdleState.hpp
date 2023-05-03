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

class PodIdleState : public State{
public:
	PodIdleState();

	void setup() override;

	void update() override;

	void dispose() override;
private:
	can::RxMessageQueue<can::messages::SENSOR_HELLO_WORLD> m_startupMessageQueue;

};

#endif /* INC_PODIDLESTATE_HPP_ */
