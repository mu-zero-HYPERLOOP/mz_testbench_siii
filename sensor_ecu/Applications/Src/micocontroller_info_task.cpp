/*
 * OnBoardSensorTask.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#ifndef SRC_ONBOARDSENSORTASK_CPP_
#define SRC_ONBOARDSENSORTASK_CPP_

#include "OnBoardSensors.hpp"
#include "peripheral_config.hpp"
#include "canzero.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

static inline size_t estimateFreeMemory() {
	return xPortGetFreeHeapSize();
}


static inline float estimateCPUusage() {
	/*
	static float g_oldIdleTick = 0;
	static float g_oldTotalTick = 0;
	float cpuUsage = 100.0f
			* (1.0f
					- ((float) (xTaskGetIdleRunTimeCounter() - g_oldIdleTick)
							/ (float) (portGET_RUN_TIME_COUNTER_VALUE()
									- g_oldTotalTick)));
	if (cpuUsage > 100.0f) {
		cpuUsage = 100.0f;
	} else if (cpuUsage < 0) {
		cpuUsage = 0;
	}
	g_oldIdleTick = xTaskGetIdleRunTimeCounter();
	g_oldTotalTick = portGET_RUN_TIME_COUNTER_VALUE();
	return cpuUsage;
	*/
	return 80.0;
}

#ifdef __cplusplus
extern "C" {
#endif

void microcontroller_info_entry(void *argv) {
	OnBoardSensors onBoardSensors(g_peripherals.m_onBoardTemperaturConfig);
	unsigned int frameCounter;
	while (true) {
		onBoardSensors.updateODs();
		if (frameCounter > 20) {
			float cpuUsage = estimateCPUusage();
			OD_CpuUsage_set(cpuUsage);
			size_t freeMemory = estimateFreeMemory();
			OD_MemFree_set(freeMemory);
			frameCounter = 0;
		}
		frameCounter++;
		osDelay(pdMS_TO_TICKS(50));	//update every 50ms because some nodes (i.e. PDU) need high frequency voltage data
	}
}

#ifdef __cplusplus
}
#endif

#endif /* SRC_ONBOARDSENSORTASK_CPP_ */
