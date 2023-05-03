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
	PodReadyToLaunchState();


	void setup() override;

	void update() override;

	void dispose() override;
private:
	can::RxMessageQueue<can::messages::SENSOR_HELLO_WORLD> m_groundStationReadyToLaunchQueue; //TODO would be bette as a PDO request.
	can::RxMessageQueue<can::messages::SENSOR_HELLO_WORLD> m_mdbReadyToLaunchQueue; //TODO would be better as a PDO request.
	can::RxMessageQueue<can::messages::SENSOR_HELLO_WORLD> m_brakeEcuReadyToLaunchQueue;
	NTCSensor m_coolingTemperatur;
	NTCSensor m_eboxTemperatur;
};

#endif /* INC_PODREADYTOLAUNCHSTATE_HPP_ */
