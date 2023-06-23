/*
 * ground_station_remote.hpp
 *
 *  Created on: May 27, 2023
 *      Author: OfficeLaptop
 */

#include "ground_station_remote.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "estdio.hpp"
#include "canzero.hpp"

namespace ground {

void reset(){
	OD_TelemetryCommands_set(COMMAND::NONE);
}

Command lastCommand(){
	return OD_TelemetryCommands_get();
}


void init(){
	OD_TelemetryCommands_set(COMMAND::NONE);
	//register receivers for
}

void update(){
	if(OD_TelemetryCommands_get() == COMMAND::EMERGENCY){
		ERR_OtherError_set();
	}
}

}
