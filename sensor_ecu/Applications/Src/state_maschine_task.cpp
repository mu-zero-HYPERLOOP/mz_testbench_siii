/*
 * state_maschine_entry.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include "GlobalState.hpp"


#ifdef __cplusplus
extern "C" {
#endif


void state_maschine_entry(void *argv) {
	GlobalState::getInstance().start();

}

#ifdef __cplusplus
}
#endif
