/*
 * cz_emergency_task.cpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */

#ifndef CANZERO_STATIC_INC_CZ_EMERGENCY_HPP_
#define CANZERO_STATIC_INC_CZ_EMERGENCY_HPP_

#include "FreeRTOS.h"
#include "cmsis_os.h"

extern osThreadId_t emergencyTaskHandle;

namespace canzero::emergency {
	void consumer_entry(void* argv);
}





#endif /* CANZERO_STATIC_INC_CZ_EMERGENCY_HPP_ */
