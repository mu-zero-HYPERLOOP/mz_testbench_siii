/*
 * cz_interface.h
 *
 *  Created on: Oct 24, 2020
 *      Author: Felix
 */

#ifndef CZ_INTERFACE_H_
#define CZ_INTERFACE_H_

#include <Heartbeat.hpp>
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "message_buffer.h"
#include "cmsis_os.h"

#include "can.h"
#include "CANZERO_OD.hpp"
#include "canzero_emcy.hpp"
#include "typedefinitions.h"
#include "log.h"

extern volatile uint8_t OD_NodeStatus;

class cz_interface{
public:
	cz_interface();
	~cz_interface();
	void init(void);

	static void cz_send(void* params);
	static void cz_receive(void *params);

	void setFilter(unsigned int canModule, CAN_FilterTypeDef canFilter);
	cz_status getStatus();
	void setStatus(cz_status);

	static cz_interface* getInstance();

private:
	static cz_interface* canInterface;

	volatile uint8_t& nodeStatus =  OD_NodeStatus;

	CAN_FilterTypeDef sFilterConfig1;
	CAN_FilterTypeDef sFilterConfig2;
};
#endif //CZ_INTERFACE_H_
