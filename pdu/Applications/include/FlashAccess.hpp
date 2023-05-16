/*
 * FlashAccess.hpp
 *
 *  Created on: 27.01.2021
 *      Author: Felix
 */

#ifndef INCLUDE_FLASHACCESS_HPP_
#define INCLUDE_FLASHACCESS_HPP_

#include "stdint.h"

uint32_t flash_read(uint32_t address);

void flash_write(uint32_t address, uint32_t data);


#endif /* INCLUDE_FLASHACCESS_HPP_ */
