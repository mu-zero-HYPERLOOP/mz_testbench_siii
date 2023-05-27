/*
 * ground_station_remote.hpp
 *
 *  Created on: May 27, 2023
 *      Author: OfficeLaptop
 */

#include "ground_station_remote.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "canzero.hpp"

namespace ground {

static volatile Command m_lastCommand = Command::COMMAND_NONE;
static volatile TickType_t m_lastCommandTime = 0;
static constexpr TickType_t COMMAND_LIFETIME = pdMS_TO_TICKS(100);

void reset(){
	m_lastCommand = Command::COMMAND_NONE;
	m_lastCommandTime = 0;
}

Command lastCommand(){
	return m_lastCommand;
}

static void groundStationCommandReceiver(RxMessage& raw){
	m_lastCommand = Command::COMMAND_LAUNCH;
	m_lastCommandTime = xTaskGetTickCount();
}

void init(){
	//register receivers for
}

void update(){
	TickType_t timeSinceLastCommand = xTaskGetTickCount() - m_lastCommandTime;
	if(timeSinceLastCommand > COMMAND_LIFETIME){
		reset();
	}
}

}
