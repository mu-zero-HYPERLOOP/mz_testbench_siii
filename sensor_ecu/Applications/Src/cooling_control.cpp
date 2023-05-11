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
static MODE s_nextMode;
static osMutexId_t s_modeMutex = osMutexNew(NULL);

void setMode(MODE mode) {
	osMutexAcquire(s_modeMutex, osWaitForever);
	s_nextMode = mode;
	osMutexRelease(s_modeMutex);
}

void update(){
	osMutexAcquire(s_modeMutex, osWaitForever);
	s_mode = s_nextMode;
	osMutexRelease(s_modeMutex);

	switch(s_mode){
	case MODE::ON:
		pdu::
		break;
	case MODE::DYNAMIC:
		break;
	case MODE::OFF:
		break;
	}

	float temperature = 0; // TODO get temperatur values.

}

}
