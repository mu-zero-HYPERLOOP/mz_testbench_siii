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

void init(){
	can::registerMessageReceiver<can::messages::CLU_TX_LevitationState>(cluStateReceiver);
	can::registerMessageReceiver<can::messages::CLU_TX_CoolingState>(cluCoolingReceiver);
}

void update(){
}

}
