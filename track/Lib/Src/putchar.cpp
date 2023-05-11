/*
 * putchar.cpp
 *
 *  Created on: Apr 25, 2023
 *      Author: OfficeLaptop
 */

#include "estdio.hpp"
#include "main.h"

void _putchar(char character) {
	ITM_SendChar(character);
}

