/*
 * PodIdleState.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include "PodIdleState.hpp"
#include "PodLaunchPreparationState.hpp"
#include "StateMaschine.hpp"

PodIdleState::PodIdleState() {
	// TODO Auto-generated constructor stub

}

void PodIdleState::setup(){

}

void PodIdleState::update(){
	if(m_startupMessageQueue.hasAny()){
		auto msg = m_startupMessageQueue.dequeue();
		//TODO parse msg.
		m_stateMaschine->setState<PodLaunchPreparationState>();
	}
}

void PodIdleState::dispose(){

}

