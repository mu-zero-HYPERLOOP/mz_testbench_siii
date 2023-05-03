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
	PodLaunchPreparationState();

	void setup() override;

	void update() override;

	void dispose() override;
private:
	//TODO fix queue duplication (maybe by changing the RxMessageQueue)
	can::RxMessageQueue<can::messages::SENSOR_HELLO_WORLD> m_mdbCapacitorPrechargeDoneQueue;
	can::RxMessageQueue<can::messages::SENSOR_HELLO_WORLD> m_trackDistanceOkQueue;
	AdcChannelController m_coolingPressure;
	NTCSensor m_coolingTemperatur;
	ImuMaster m_imuMaster;
	SDC m_sdc;
};

#endif /* INC_PODLAUNCHPREPARATIONSTATE_HPP_ */
