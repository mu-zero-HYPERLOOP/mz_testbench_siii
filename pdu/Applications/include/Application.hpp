#ifndef APPLICATION_H
#define APPLICATION_H

#include "cmsis_os.h"

class Application {
public:
	Application();
	//static void vDemoTask(void *vParams);
	void setPriority(osPriority_t);
	void setStackSize(int);

	osThreadId_t create(const char* taskName, osThreadFunc_t func, void* attr);

	void setName(const char* name);
	TaskHandle_t getTaskHandle();


	osThreadId_t taskHandle = NULL;

	osThreadAttr_t taskAttributes; //private?

};


#endif //APPLICATION_H

