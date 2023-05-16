/*
 * Watchdog.hpp
 *
 *  Created on: 11.05.2021
 *      Author: Florian
 */

#ifndef INCLUDE_WATCHDOG_HPP_
#define INCLUDE_WATCHDOG_HPP_

#include <cstdint>

class Watchdog {
private:
	uint32_t m_lastWatchdogReset = 0;
	uint32_t m_timeoutMs = 0;
public:
	Watchdog(uint32_t timeoutMs);
	virtual ~Watchdog();

	void reset();
	bool isTimedOut();
};

#endif /* INCLUDE_WATCHDOG_HPP_ */
