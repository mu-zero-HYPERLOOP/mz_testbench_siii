#include <Application.hpp>
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

Application::Application(){
	/* In FreeRTOS stack is not in bytes, but in sizeof(StackType_t) which is 4 on ARM ports.       */
	/* Stack size should be therefore 4 byte aligned in order to avoid division caused side effects */
	taskAttributes.stack_size=256 * 4;
	taskAttributes.priority = (osPriority_t) osPriorityNormal;
	//this attributes must be NULL / 0 and are only used for static memory allocation
	taskAttributes.cb_mem = NULL;
	taskAttributes.stack_mem = NULL;
	taskAttributes.cb_size = 0;
	taskAttributes.attr_bits = 0;
#ifdef DEBUG
	printf("new application!\n");
#endif
}

osThreadId_t Application::create(const char* taskName, osThreadFunc_t func, void* attr)
{
	setName(taskName);
	taskHandle = osThreadNew(func, attr, &taskAttributes);
	if(taskHandle==NULL){
#ifdef DEBUG
	printf("task creation failed (most likely EOM or false parameter)!\n");
#endif
	}

	return taskHandle;
}

void Application::setName(const char* name){
	taskAttributes.name=name;
}

void Application::setStackSize(int stacksize){
	taskAttributes.stack_size=stacksize;
}

void Application::setPriority(osPriority_t prio){
	if(taskHandle==NULL){
		taskAttributes.priority = (osPriority_t) prio;
	}else{
		osThreadSetPriority(taskHandle, prio);
	}
}

TaskHandle_t Application::getTaskHandle(){
	return (TaskHandle_t) taskHandle;
}

