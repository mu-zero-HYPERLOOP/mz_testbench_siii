/*
 * cooling_controll.hpp
 *
 *  Created on: May 20, 2023
 *      Author: OfficeLaptop
 */

#include "cooling_controll.hpp"

#include "mdb_remote.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "canzero.hpp"

namespace cooling_controll {

static float mdbTemperatureThreshold = 40;
constexpr TickType_t MIN_TIME_BETWEEN_MSG = 500;  //timeout between can messages to reduce software overhead.
static TickType_t lastCoolingMsg;
static bool coolingStatus = false;

void init() {
	lastCoolingMsg = 0;
}


void setCoolingThreshold(float temperature){
	mdbTemperatureThreshold = temperature;
}


void update() {
	bool requireCooling = false;
	for (uint8_t i = 0; i < mdb::MDB_COUNT; i++) {
		if (mdb::getTemperature(i) > mdbTemperatureThreshold) {
			requireCooling = true;
		}
	}
	if (requireCooling != coolingStatus) {
		coolingStatus = requireCooling;
		lastCoolingMsg = 0; //enfore instant msg.
	}
	TickType_t ticksSinceLastMsg = xTaskGetTickCount() - lastCoolingMsg;
	if (ticksSinceLastMsg > MIN_TIME_BETWEEN_MSG) {
		lastCoolingMsg = xTaskGetTickCount();
		can::Message<can::messages::CLU_TX_CoolingState> msg;
		msg.set<can::signals::CLU_RequiresCooling>(requireCooling);
		msg.send();
	}
}

}
