/*
 * cz_statemachine.hpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */

#ifndef CANZERO_STATIC_INC_CZ_STATEMACHINE_HPP_
#define CANZERO_STATIC_INC_CZ_STATEMACHINE_HPP_

#include "cz_typedefinitions.hpp"

namespace canzero {

void init();
void setStatus(cz_status status);
cz_status getStatus();

}

#endif /* CANZERO_STATIC_INC_CZ_STATEMACHINE_HPP_ */
