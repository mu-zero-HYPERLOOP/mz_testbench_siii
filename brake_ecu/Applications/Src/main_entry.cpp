/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#include "GlobalPeripheralRegistry.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "GlobalStateReceiver.hpp"
#include "estdio.hpp"


#ifdef __cplusplus
extern "C" {
#endif


void main_entry(void *argv) {
	while(true){
		osDelay(pdMS_TO_TICKS(1000));

	}
}

#ifdef __cplusplus
}
#endif
