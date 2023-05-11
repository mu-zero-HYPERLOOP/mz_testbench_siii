/*
 * cooling_control.hpp
 *
 *  Created on: 11 May 2023
 *      Author: karl
 */


#include "cooling_controll.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "pdu_control.hpp"

namespace cooling {

constexpr float THRESHOLD = 30;

static MODE s_mode;
static MODE s_nextMode = MODE::DYNAMIC;
static osMutexId_t s_modeMutex = osMutexNew(NULL);


pdu::HpChannel COOLING_PUMP_CHANNEL = pdu::HP_CHANNEL1;

void setMode(MODE mode) {
	osMutexAcquire(s_modeMutex, osWaitForever);
	s_nextMode = mode;
	osMutexRelease(s_modeMutex);
}

bool toggle = true;

void update(){
	osMutexAcquire(s_modeMutex, osWaitForever);
	s_mode = s_nextMode;
	osMutexRelease(s_modeMutex);

	switch(s_mode){
	case MODE::ON:
		pdu::enableChannel(COOLING_PUMP_CHANNEL);
		break;
	case MODE::DYNAMIC:
		if(toggle){
			pdu::enableChannel(COOLING_PUMP_CHANNEL);
		}else{
			pdu::disableChannel(COOLING_PUMP_CHANNEL);
		}
		toggle = !toggle;
		break;
	case MODE::OFF:
		pdu::disableChannel(COOLING_PUMP_CHANNEL);
		break;
	}

}

}
