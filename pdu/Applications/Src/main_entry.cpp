/*
 * main_entry.cpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */


#include "NewPDU.hpp"
#include "PDU.hpp"

#ifdef __cplusplus
extern "C" {
#endif


void main_entry(void *argv) {
	pduAppFunction(argv);

}

#ifdef __cplusplus
}
#endif
