/**
 * @file TaskManager.cpp
 * @brief Contains the Task Manager function prototypes
 *
 */
#ifndef _TaskManager_
#define _TaskManager_

#include "FreeRTOS.h"
#include "cmsis_os.h"

/**
 * @namespace stats
 * contains all the functions of the Task Manager.
 */
namespace stats{

	void printTaskList();
	void printCPUusage();
	void updateSensorStats(void*);
	void estimateFreeMemory();
	void estimateCPUusage();

}
#endif
