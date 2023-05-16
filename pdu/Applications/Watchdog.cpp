/*
 * Watchdog.cpp
 *
 *  Created on: 11.05.2021
 *      Author: Florian
 */

#include <Watchdog.hpp>
#include "FreeRTOS.h"
#include "cmsis_os.h"

/**
 * Constructor for Watchdog class
 * @param timeoutMs Watchdog timeout in milliseconds
 */
Watchdog::Watchdog(uint32_t timeoutMs) : m_timeoutMs{ timeoutMs } {
}

Watchdog::~Watchdog() {
	// TODO Auto-generated destructor stub
}

/**
 * Reset the watchdog
 */
void Watchdog::reset() {
	m_lastWatchdogReset = xTaskGetTickCount();
}

/**
 * Check whether the watchdog timed out.
 * @return true if there was a timeout
 */
bool Watchdog::isTimedOut() {
	if(xTaskGetTickCount() - m_lastWatchdogReset > m_timeoutMs) {
		return true;
	} else {
		return false;
	}
}
