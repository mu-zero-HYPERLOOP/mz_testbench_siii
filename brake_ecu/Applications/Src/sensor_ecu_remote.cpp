/*
 * sensor_ecu_remote.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "sensor_ecu_remote.hpp"
#include "canzero.hpp"
#include <cinttypes>
#include "State.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"


namespace sensor_ecu {

static volatile PodState state = STATE::POD_OFF;

static constexpr TickType_t COOLING_PRESSURE_SEND_INTERVAL = pdMS_TO_TICKS(50);
static TickType_t lastMsgTime = 0;

void statusReceiver(RxMessage& raw){
	can::Message<can::messages::SensorF_TX_StatePod> msg{raw};
	state = msg.get<can::signals::SensorF_TX_PodState>();
}

void init(){
	can::registerMessageReceiver<can::messages::SensorF_TX_StatePod>(statusReceiver);
}

void update(){
	TickType_t timeSinceLastMsg = xTaskGetTickCount() - lastMsgTime;
	if(timeSinceLastMsg > COOLING_PRESSURE_SEND_INTERVAL){
		can::Message<can::messages::BrakeF_TX_PressureCooling> coolingMsg;
		coolingMsg.set<can::signals::BrakeF_TX_Pressure_Reservoir>(OD_CoolingPressure_get());
		coolingMsg.send();
		lastMsgTime = xTaskGetTickCount();
	}
}

PodState getState(){
	return state;
}

}
