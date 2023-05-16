/*
 * FlashAccess.cpp
 *
 *  Created on: 27.01.2021
 *      Author: Felix
 */

#include "FlashAccess.hpp"
#include "stm32f4xx_hal.h"

uint32_t flash_read(uint32_t address){
    return *(uint32_t*)address;
}

void flash_write(uint32_t address, uint32_t data){
    HAL_FLASH_Unlock();
    FLASH_Erase_Sector(FLASH_SECTOR_11,VOLTAGE_RANGE_1);
    if(0x080E0000<=address && address<=0x080FFFFF){
    	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,address,data);
    	HAL_FLASH_Lock();
    }
    else{
    	HAL_FLASH_Lock();
    	while(1);
    }

}

