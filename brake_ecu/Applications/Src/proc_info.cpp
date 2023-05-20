/*
 * info_controll.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include <proc_info.hpp>
#include "AdcChannelController.hpp"
#include <cinttypes>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include <cmath>
#include <cstring>
#include "canzero.hpp"
#include "MovingAverageFilter.hpp"

namespace proc_info {

AdcChannelController internalTemperatureAdc;
AdcChannelController externalTemperatureAdc;
AdcChannelController inputVoltageAdc;
MovingAverageFilter<10> internalTemperatureFilter(30);
MovingAverageFilter<10> externalTemperatureFilter(30);
MovingAverageFilter<10> inputVoltageFilter(24);

uint16_t avalue;

unsigned int frameCounter;

void init() {
	internalTemperatureAdc = AdcChannelController(ADC_MODULE1, 2);
	externalTemperatureAdc = AdcChannelController(ADC_MODULE1, 0);
	inputVoltageAdc = AdcChannelController(ADC_MODULE1, 1);

}

void update() {
	// read sensor data.
	avalue = internalTemperatureAdc.get();
	float internalTemp = (3.3f * (float) avalue / 4095.0f - 0.76f) / 0.0025f + 25.0f;

	avalue = externalTemperatureAdc.get();
	float externalTemp = 1.0f / (1.0f / 298.15f + 1.0f / 3380.0f * logf(1.0f / (4095.0f / (float) avalue - 1.0f))) - 273.15f;

	avalue = inputVoltageAdc.get();
	float inputVoltage = (float) avalue / 4095.0f * 3.3f / 0.106464f + 0.6f;

	// filter sensor data.

	internalTemperatureFilter.addValue(internalTemp);
	externalTemperatureFilter.addValue(externalTemp);
	inputVoltageFilter.addValue(inputVoltage);

	float filteredInternalTemperature = internalTemperatureFilter.get();
	float filteredExternalTemperature = externalTemperatureFilter.get();
	float filteredInputVoltage = inputVoltageFilter.get();

	// update ods.
	OD_BoardTemp_set(
			(filteredExternalTemperature + filteredInternalTemperature)
					* 0.5);
	OD_InputVoltage_set(filteredInputVoltage);

	// every 20 iterations estimate cpu resources.
	if (frameCounter > 20) {

		// estimate cpu usage by finding cpu usage of idle task.
		UBaseType_t numTasks = uxTaskGetNumberOfTasks();
		TaskStatus_t *pxTaskStatusArray =
				reinterpret_cast<TaskStatus_t*>(malloc(
						sizeof(TaskStatus_t) * numTasks));
		if (pxTaskStatusArray != NULL) {
			unsigned long ulTotalRuntime;
			uxTaskGetSystemState(pxTaskStatusArray, numTasks,
					&ulTotalRuntime);

			//linear search for the idle task.
			for (size_t i = 0; i < numTasks; i++) {
				if (strcmp(pxTaskStatusArray[i].pcTaskName, "IDLE") == 0) {
					float runtime_percent = (float) (100
							* (float) pxTaskStatusArray[i].ulRunTimeCounter
							/ (float) ulTotalRuntime);
					float cpuUsage = 100.0 - runtime_percent;
					if (cpuUsage > 100.0f) {
						cpuUsage = 100.0f;
					} else if (cpuUsage < 0) {
						cpuUsage = 0;
					}
					OD_CpuUsage_set(cpuUsage);
					break;
				}
			}
			free(pxTaskStatusArray);
		}

		// estimate free memory.
		size_t freeMemory = xPortGetFreeHeapSize();
		OD_MemFree_set(freeMemory);

		frameCounter = 0;
	}
	frameCounter++;
}

}

#include "tim.h"
// IMPORTANT!!!!.
// for runtime stats to follow this tutorial there are some changes in stm32fxx_it.c required.
// https://stm32world.com/wiki/STM32_FreeRTOS_Statistics

// global required for the overwrites and isrs.
volatile unsigned long ulHighFrequencyTimerTicks;

// FreeRTOS overwrites required to fetch runtime stats.
void configureTimerForRunTimeStats(void)
{
    ulHighFrequencyTimerTicks = 0;
    HAL_TIM_Base_Start_IT(&htim10);
}

unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}
