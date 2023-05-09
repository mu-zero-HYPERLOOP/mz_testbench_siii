/*
 * GlobalState.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include <PodStopLevitationState.hpp>
#include "canzero.hpp"
#include "PodState.hpp"
#include "PodEmergencyState.hpp"
#include "PodIdleState.hpp"
#include "PodLaunchPreparationState.hpp"
#include "PodLaunchingState.hpp"
#include "PodReadyToLaunchState.hpp"
#include "PodStartLevitationState.hpp"
#include "PodSafeToApproach.hpp"
#include "PodStartupState.hpp"
#include "State.hpp"
#include "StateMaschine.hpp"
#include "PodBreakState.hpp"



class GlobalState {
public:
	static GlobalState& getInstance() {
		static GlobalState instance;
		return instance;
	}

	template<typename STATE>
	void setState(){
		const State& state = m_stateMaschine.getState<STATE>();
		broadcastState(state.getPodState());
		m_stateMaschine.setState<STATE>();
	}

	void start(){
		broadcastState(m_startupState.getPodState());
		m_stateMaschine.start<PodStartupState>();
	}

	~GlobalState() = default;
	GlobalState(GlobalState&) = delete;
	GlobalState(GlobalState&&) = delete;
	GlobalState& operator=(GlobalState&) = delete;
	GlobalState& operator=(GlobalState&&) = delete;

private:

	void broadcastState(PodState state) {
		can::Message<can::messages::SensorF_TX_StatePod> msg;
		msg.set<can::signals::SensorF_TX_PodState>(static_cast<uint8_t>(state));
		msg.set<can::signals::SensorF_TX_PodState_Last>(static_cast<uint8_t>(state));
		msg.set<can::signals::SensorF_TX_PodState_Target>(static_cast<uint8_t>(state));
		msg.send();
	}

	explicit GlobalState() : m_stateMaschine(&m_stateMemory){
		m_stateMaschine.registerState(m_startupState);
		m_stateMaschine.registerState(m_idleState);
		m_stateMaschine.registerState(m_readyToLaunch);
		m_stateMaschine.registerState(m_launchPrep);
		m_stateMaschine.registerState(m_podStartLevitation);
		m_stateMaschine.registerState(m_launchingState);
		m_stateMaschine.registerState(m_stopState);
		m_stateMaschine.registerState(m_breakState);
		m_stateMaschine.registerState(m_safeState);
		m_stateMaschine.registerState(m_emcyState);
	}


	StateMaschineMemory<15> m_stateMemory;
	StateMaschine m_stateMaschine;
	PodStartupState m_startupState;
	PodIdleState m_idleState;
	PodReadyToLaunchState m_readyToLaunch;
	PodLaunchPreparationState m_launchPrep;
	PodStartLevitation m_podStartLevitation;
	PodLaunchingState m_launchingState;
	PodStopLevitationState m_stopState;
	PodBreakState m_breakState;
	PodSafeToApproach m_safeState;
	PodEmergencyState m_emcyState;
};
