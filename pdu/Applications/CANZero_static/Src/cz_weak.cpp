/*
 * cz_weak.cpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */

#ifndef CANZERO_STATIC_SRC_CZ_WEAK_CPP_
#define CANZERO_STATIC_SRC_CZ_WEAK_CPP_

#include "cz_weak.hpp"


void __attribute__((weak)) canzero::handle_emergency_warning(){
	//TODO missing default implementation.
}


void __attribute__((weak)) canzero::handle_heartbeat_miss(){
	//TODO missing default implementation.
}

void __attribute__((weak)) canzero::handle_txmailbox_overflow(CAN_HandleTypeDef* hcan){
	//TODO missing default implementation.
}

void __attribute__((weak)) canzero::handle_trasmission_request_error(){
	//TODO missing default implementation.
}



#endif /* CANZERO_STATIC_SRC_CZ_WEAK_CPP_ */
