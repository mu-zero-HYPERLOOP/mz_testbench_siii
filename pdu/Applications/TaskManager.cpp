/**
 * @file TaskManager.cpp
 * @brief Contains the Task Manager functions
 *
 */
#include "TaskManager.hpp"
#include "log.h"
#include "adc.h"
#include "AdcDma.hpp"
#include "canzero_od.hpp"

// ADC1 reads the four analog inputs on the BCU (Board temperature (IN10), Input voltage (IN12), Temperature Sensor, Vrefint)
AdcDma<4> adc1{&hadc1};

namespace stats{
	TickType_t oldIdleTick=0;
	TickType_t oldTotalTick=0;
}
/**
 * @brief prints all Task with their current status to the console
 *
 */
void stats::printTaskList(){
	char* taskListBuffer = new char;
	vTaskList(taskListBuffer);
	printDebug(taskListBuffer);
	delete(taskListBuffer);
}

/**
 * @brief prints the current CPU load of every task
 *
 */
void stats::printCPUusage(){
	char* taskListBuffer2 = new char;
	vTaskGetRunTimeStats(taskListBuffer2);
	printDebug(taskListBuffer2);
	delete(taskListBuffer2);
}

/**
 * @brief
 *
 */
void stats::estimateFreeMemory(){
	OD_MemFree_set(xPortGetFreeHeapSize());
}

void stats::estimateCPUusage(){
	float cpuUsage = 100.0f * ( 1.0f - ( (float) (xTaskGetIdleRunTimeCounter()-stats::oldIdleTick) / (float) (portGET_RUN_TIME_COUNTER_VALUE()-stats::oldTotalTick) ) );
	if(cpuUsage > 100.0f) {
		cpuUsage = 100.0f;
	} else if(cpuUsage < 0) {
		cpuUsage = 0;
	}
	OD_CpuUsage_set(cpuUsage);
	stats::oldIdleTick = xTaskGetIdleRunTimeCounter();
	stats::oldTotalTick = portGET_RUN_TIME_COUNTER_VALUE();
}

/**
 * @brief updates all Board Sensors
 * calculates the board temperature and input voltage and updates the corresponding values in the OD: OD_BoardTemp and OD_InputVoltage
 *
 * @param task parameter (unused)
 */
void stats::updateSensorStats(void*){
	uint8_t cyclesCounter = 0;

	// Init AdcDma, get handle of this task for later unblocking. ADC will read four channels. They need to be configured in CubeMx
	adc1.init(10);		// Read each channel ten times and average

	while(1){
		// Start ADC with DMA
		adc1.start();

		// Wait until the ADC finished reading all channels.
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

		// Get data pointer. Order of values is as configured in CubeMX
		float* adcData = adc1.getData();

		// NTC on BCU V1 is NTCG163JF103FT1 (10kOhms at 25°C. Beta = 3435K
		// The NTC is connected to Gnd and has a R1=10kOhms Pull-Up resistor
		// 1) Using voltage divider equation to convert voltage into R_NTC: R_NTC = R1 * adcValue / (4096 - adcValue)
		// 2) Using NTC equation to get temperature

		// Internal temperature sensor of STM32
		// ADC sampling time should be at least 10us: Our ADC runs at 84MHz/4=21MHz -> at least 210 samples -> 480 samples were selected
		float ntcTemperature = 1.0f / (1.0f / 298.15f + 1.0f / 3380.0f * logf(1.0f / (4095.0f / (float) adcData[0] - 1.0f) )) - 273.15f;
		float inputVoltage = (float)adcData[1]/ 4095.0f * 3.3f / 0.106464f + 0.6f;
		float internalTemp = (3.3f * (float) adcData[2]/4095.0f - 0.76f) / 0.0025f + 25.0f;

		OD_BoardTemp_set((ntcTemperature + internalTemp) / 2.0f);
		OD_InputVoltage_set(inputVoltage);

		// Update CPU usage only every twenty cycles, so every second
		cyclesCounter++;
		if(cyclesCounter >= 20) {
			cyclesCounter = 0;
			estimateCPUusage();
			estimateFreeMemory();
		}

		//osThreadSuspend(runtimeStatsHandle);
		osDelay(pdMS_TO_TICKS(50));	//update every 50ms because some nodes (i.e. PDU) need high frequency voltage data
	}
}
