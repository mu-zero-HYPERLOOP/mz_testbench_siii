/*
 * log.h
 *
 *  Created on: 29.11.2020
 *      Author: Felix
 */

#ifndef INCLUDE_LOG_H_
#define INCLUDE_LOG_H_

#include <stdio.h>
#include "cmsis_os.h"

//TODO: find a better solution for printing, because currently interrupts are disabled during printing
#ifdef DEBUG

#define printDebug(x, ...)  {\
		taskENTER_CRITICAL();\
	printf(x, ##__VA_ARGS__);\
	taskEXIT_CRITICAL();}

#define printDebugISR(x, ...)  {\
		UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();\
	printf(x, ##__VA_ARGS__);\
	taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);}

#else //DEBUG
#define printDebug(x, ...)
#endif //DEBUG

#endif /* INCLUDE_LOG_H_ */
