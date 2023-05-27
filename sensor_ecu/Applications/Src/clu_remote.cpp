/*
 * clu_remote.hpp
 *
 *  Created on: May 21, 2023
 *      Author: OfficeLaptop
 */

#include "clu_remote.hpp"

#include "canzero.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"

namespace clu {

static volatile State m_state;
static volatile bool m_requiresCooling;

static Action m_requestedAction = MDB_STOP;
static bool m_actionDirty = true;
static TickType_t m_lastActionRequest = 0;
static constexpr TickType_t MAX_TIME_BETWEEN_ACTION_REQUESTS = pdMS_TO_TICKS(50);

State getState(){
	return m_state;
}

bool requiresCooling(){
	return m_requiresCooling;
}

void cluCoolingReceiver(RxMessage& raw){
	can::Message<can::messages::CLU_TX_CoolingState> msg{raw};
	m_requiresCooling = msg.get<can::signals::CLU_RequiresCooling>();
}

void cluStateReceiver(RxMessage& raw){
	can::Message<can::messages::CLU_TX_LevitationState> msg {raw};
	m_state = static_cast<State>(msg.get<can::signals::CLU_LevitationState>());
}


void requestAction(Action action){
	if(action != m_requestedAction){
		m_requestedAction = action;
		m_actionDirty = true;
	}
}

void init(){
	can::registerMessageReceiver<can::messages::CLU_TX_LevitationState>(cluStateReceiver);
	can::registerMessageReceiver<can::messages::CLU_TX_CoolingState>(cluCoolingReceiver);
}

void update(){
	TickType_t timeSinceLastActionRequest = xTaskGetTickCount() - m_lastActionRequest;
	if(m_actionDirty || timeSinceLastActionRequest > MAX_TIME_BETWEEN_ACTION_REQUESTS){
		//TODO check if sending is required based on m_state


		//send action
		m_lastActionRequest = xTaskGetTickCount();
		m_actionDirty = false;
	}
}

}
