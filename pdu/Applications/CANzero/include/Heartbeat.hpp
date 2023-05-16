/*
 * Heartbeat.h
 *
 *  Created on: 02.12.2020
 *      Author: Felix
 */

#ifndef CANZERO_HEARTBEAT_H_
#define CANZERO_HEARTBEAT_H_
#include <cz_interface.hpp>
#include "stdint.h"

extern volatile uint16_t OD_HeartbeatInterval;

class Heartbeat {
public:
	Heartbeat();
	virtual ~Heartbeat();
	static void sendHeartbeat(void*);
	static void consumeHeartbeat(void*);
	static Heartbeat* getHeartbeatInstance();
	void OD_HeartbeatInterval_set(const uint16_t value);
	uint16_t OD_HeartbeatInterval_get();

private:
	volatile uint16_t& interval = OD_HeartbeatInterval;
	static Heartbeat* hb;
};

#endif /* CANZERO_HEARTBEAT_H_ */
