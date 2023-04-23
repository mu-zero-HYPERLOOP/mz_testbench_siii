/*
 * testbench.cpp
 *
 *  Created on: Apr 17, 2023
 *      Author: karl
 */

//fixme includes
#include <log_dep.hpp>
#include "AdcDmaController.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "adc.h"
#include "GPIOExtiController.hpp"
#include "GPIOWriteController.hpp"
#include "GPIOReadController.hpp"
#include "canzero.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void testbench_entry(void* argv){
	while(true){
		RxMessage message;
		if(can::checkRxMessage<can::messages::SENSOR_HELLO_WORLD>(message)){
			logln("Hello World!");
		}
		osDelay(500);
	}
}

#ifdef __cplusplus
}
#endif
