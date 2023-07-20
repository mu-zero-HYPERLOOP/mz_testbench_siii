/*
 * error_prop.hpp
 *
 *  Created on: Jun 18, 2023
 *      Author: OfficeLaptop
 */

#include "error_prop.hpp"
#include "canzero.hpp"
#include <cmath>
#include "sensor_ecu_remote.hpp"

constexpr TickType_t ERROR_SEND_INTERVAL = pdMS_TO_TICKS(50);
TickType_t lastErrorMsg = 0;

constexpr uint8_t CPU_OVER_TEMP_PRIO = 10;
constexpr uint8_t OVER_VOLT_PRIO = 10;
constexpr uint8_t UNDER_VOLT_PRIO = 8;

constexpr uint8_t LEVI_ERROR_PRIO_LEVI = 20;
constexpr uint8_t LEVI_ERROR_PRIO_PREP = 10;
constexpr uint8_t LEVI_ERROR_PRIO_NON_LEVI = 5;

constexpr uint8_t MAGNET_OVER_TEMP_PRIO_LEVI = 5;
constexpr uint8_t MAGNET_OVER_TEMP_PRIO_PREP = 5;
constexpr uint8_t MAGNET_OVER_TEMP_PRIO_NON_LEVI = 5;

namespace error_prop {

void init(){
	can::Message<can::messages::CLU_EMCY> msg;
	msg.send();
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
		if(ERR_MagnetOverTemp_get()){
			PodState state = sensor_ecu_remote::getState();
			if(state == STATE::POD_STARTUP || state == STATE::POD_IDLE
					|| state == STATE::POD_SAFE_TO_APPROCH || state == STATE::POD_END_OF_RUN
					|| state == STATE::POD_ROLLING || state == STATE::POD_PUSHABLE
					|| state == STATE::POD_OFF || state == STATE::POD_ENGAGE_BRAKES){
				errorPrio = std::max(MAGNET_OVER_TEMP_PRIO_NON_LEVI, errorPrio);
			}else if(state == STATE::POD_LAUNCH_PREPARATION || state == STATE::POD_READY_TO_LAUNCH){
				errorPrio = std::max(MAGNET_OVER_TEMP_PRIO_PREP, errorPrio);
			}else if(state == STATE::POD_START_LEVITATION
					|| state == STATE::POD_STABLE_LEVITATION
					|| state == STATE::POD_DISENGAGE_BRAKES
					|| state == STATE::POD_STOP_LEVITATION){

				errorPrio = std::max(MAGNET_OVER_TEMP_PRIO_LEVI, errorPrio);
			}
		}
		if(ERR_LevitationError_get()){
			PodState state = sensor_ecu_remote::getState();
			if(state == STATE::POD_STARTUP || state == STATE::POD_IDLE
					|| state == STATE::POD_SAFE_TO_APPROCH || state == STATE::POD_END_OF_RUN
					|| state == STATE::POD_ROLLING || state == STATE::POD_PUSHABLE
					|| state == STATE::POD_OFF || state == STATE::POD_ENGAGE_BRAKES){
				errorPrio = std::max(LEVI_ERROR_PRIO_NON_LEVI, errorPrio);
			}else if(state == STATE::POD_LAUNCH_PREPARATION || state == STATE::POD_READY_TO_LAUNCH){
				errorPrio = std::max(LEVI_ERROR_PRIO_PREP, errorPrio);
			}else if(state == STATE::POD_START_LEVITATION
					|| state == STATE::POD_STABLE_LEVITATION
					|| state == STATE::POD_DISENGAGE_BRAKES
					|| state == STATE::POD_STOP_LEVITATION){

				errorPrio = std::max(LEVI_ERROR_PRIO_LEVI, errorPrio);
			}
		}
		can::Message<can::messages::CLU_TX_Error> msg;
		msg.set<can::signals::CLU_TX_ErrorPrio>(errorPrio);
		msg.send();
	}

}

}
