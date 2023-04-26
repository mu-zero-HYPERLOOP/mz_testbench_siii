/*
 * cz_weak.hpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */

#ifndef CANZERO_STATIC_INC_CZ_WEAK_HPP_
#define CANZERO_STATIC_INC_CZ_WEAK_HPP_

#include "can.h"

namespace canzero{

void handle_emergency_warning();
void handle_heartbeat_miss();
void handle_txmailbox_overflow(CAN_HandleTypeDef* hcan);
void handle_trasmission_request_error();
}


#endif /* CANZERO_STATIC_INC_CZ_WEAK_HPP_ */
