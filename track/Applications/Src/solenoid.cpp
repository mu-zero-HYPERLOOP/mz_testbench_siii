/*
 * solenoid.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "solenoid.hpp"
#include "GPIOWriteController.hpp"
#include "gpio.h"
#include "canzero.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "canzero.hpp"

namespace solenoid {


GPIOWriteController solenoidGpio(DOUT1_GPIO_Port, DOUT1_Pin);

constexpr TickType_t MIN_TICKS_UNTIL_RETRACKT = pdMS_TO_TICKS(2000);
constexpr TickType_t MIN_TICKS_UNTIL_PUSH = pdMS_TO_TICKS(1000);

static TickType_t pushTime = 0;
static TickType_t retracktTime = 0;

void init(){
	solenoidGpio.reset();
	retracktTime = xTaskGetTickCount();
	OD_PistonStatus_set(Ready);
}

bool push(){
	if(OD_PistonStatus_get() == Extended)return false;
	TickType_t timeSinceRetrackt = xTaskGetTickCount() - retracktTime;
	if(timeSinceRetrackt <= MIN_TICKS_UNTIL_PUSH){
		return false;
	}
	printf("push\n");
	pushTime = xTaskGetTickCount();
	solenoidGpio.set();
	OD_PistonStatus_set(Extended);
	return true;
}

void update(){
	if(OD_PistonStatus_get() == Extended){
		TickType_t timeSincePush = xTaskGetTickCount() - pushTime;
		if(timeSincePush > MIN_TICKS_UNTIL_RETRACKT){
			pushTime = 0;
			// retrackt piston.
			printf("retrac\n");
			solenoidGpio.reset();
			retracktTime = xTaskGetTickCount();
			OD_PistonStatus_set(Ready);
		}
	}
}

}
