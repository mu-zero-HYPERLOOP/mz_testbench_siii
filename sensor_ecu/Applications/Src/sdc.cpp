/*
 * sdc.hpp
 *
 *  Created on: May 21, 2023
 *      Author: OfficeLaptop
 */

#include "sdc.hpp"
#include "GPIOWriteController.hpp"
#include "gpio.h"
#include "estdio.hpp"
#include "pdu_remote.hpp"

namespace sdc {

pdu::LpChannel SDC_CHANNEL = pdu::LpChannel::LP_CHANNEL6;

void open(){
	pdu::disableChannel(SDC_CHANNEL);
}

void close(){
	pdu::enableChannel(SDC_CHANNEL);
}


void init(){
	open();
}

void update(){
}


}
