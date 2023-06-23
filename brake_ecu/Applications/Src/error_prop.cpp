/*
 * error_prop.hpp
 *
 *  Created on: Jun 18, 2023
 *      Author: OfficeLaptop
 */

#include "error_prop.hpp"
#include "canzero.hpp"
#include <cmath>

constexpr TickType_t ERROR_SEND_INTERVAL = pdMS_TO_TICKS(50);
TickType_t lastErrorMsg = 0;

constexpr uint8_t CPU_OVER_TEMP_PRIO = 10;
constexpr uint8_t OVER_VOLT_PRIO = 10;
constexpr uint8_t UNDER_VOLT_PRIO = 8;

constexpr uint8_t OVER_PRESSURE_PRIO = 10;
constexpr uint8_t UNDER_PRESSURE_PRIO = 2;

namespace error_prop {

void init(){

}

void update(){
	TickType_t timeSinceLastErrorMsg = xTaskGetTickCount() - lastErrorMsg;
	if(timeSinceLastErrorMsg > ERROR_SEND_INTERVAL){
		lastErrorMsg = xTaskGetTickCount();
		uint8_t errorPrio = 0;
		if(ERR_CPUOverTemp_get()){
			errorPrio = std::max(CPU_OVER_TEMP_PRIO, errorPrio);
		}
		if(ERR_OverVolt_get()){
			errorPrio = std::max(OVER_VOLT_PRIO, errorPrio);
		}
		if(ERR_UnderVolt_get()){
			errorPrio = std::max(UNDER_VOLT_PRIO, errorPrio);
		}
		if(ERR_IntakeOverPressure_get() || ERR_OuttakeOverPressure_get()){
			errorPrio = std::max(OVER_PRESSURE_PRIO, errorPrio);
		}
		if(ERR_IntakeUnderPressure_get() || ERR_OuttakeUnderPressure_get()){
			errorPrio = std::max(UNDER_PRESSURE_PRIO, errorPrio);
		}
		can::Message<can::messages::BrakeF_TX_Error> msg;
		msg.set<can::signals::BrakeF_TX_ErrorPrio>(errorPrio);
		msg.send();
	}

}

}
