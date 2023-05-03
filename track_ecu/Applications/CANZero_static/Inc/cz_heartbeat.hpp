/*
 * cz_heartbeat.hpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */

#ifndef CANZERO_STATIC_INC_CZ_HEARTBEAT_HPP_
#define CANZERO_STATIC_INC_CZ_HEARTBEAT_HPP_

#include <cinttypes>

namespace canzero::heartbeat {

void producer_entry(void* argv);

void consumer_entry(void* argv);

void setInterval(uint16_t value);

uint16_t getInterval();

}


#endif /* CANZERO_STATIC_INC_CZ_HEARTBEAT_HPP_ */
