/*
 * brake_controll.cpp
 *
 *  Created on: May 14, 2023
 *      Author: OfficeLaptop
 */


#include <brake_ecu_remote.hpp>
#include "canzero.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"


namespace brake {

static bool m_brakeState;
static bool m_brakeStateDirty = true;
static TickType_t m_lastBrakeStateMsg = 0;
static constexpr TickType_t MAX_TIME_BETWEEN_BRAKE_STATE_MSG = pdMS_TO_TICKS(50);

void pressureReceiver(RxMessage& raw){
	can::Message<can::messages::BrakeF_TX_PressureCooling> msg{raw};
	OD_CoolingPressure_set(msg.get<can::signals::BrakeF_TX_Pressure_Reservoir>());
}


void disengageBrakes(){
	if(m_brakeState){
		m_brakeState = false;
		m_brakeStateDirty = true;
	}
}

void engageBrakes(){
	if(not m_brakeState){
		m_brakeState = true;
		m_brakeStateDirty  = true;
	}
}

void init(){
	can::registerMessageReceiver<can::messages::BrakeF_TX_PressureCooling>(pressureReceiver);
}

void update(){
	TickType_t timeSinceLastBrakeMsg = xTaskGetTickCount() - m_lastBrakeStateMsg;
	if(timeSinceLastBrakeMsg > MAX_TIME_BETWEEN_BRAKE_STATE_MSG){
		m_brakeStateDirty  = true;
	}
	if(m_brakeStateDirty){
		//send can message to brake ecu.
		m_lastBrakeStateMsg = xTaskGetTickCount();
		m_brakeStateDirty = false;
	}
}

}
