/*
 * cz_statemaschine.cpp
 *
 *  Created on: Apr 20, 2023
 *      Author: OfficeLaptop
 */

#include "cz_statemachine.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "dbc_parser.hpp"

static const osMutexAttr_t nodeStateMutexAttr = {"nodeStateMutex",(osMutexPrioInherit|osMutexRobust),NULL,0,};
static osMutexId_t nodeStateMutex = osMutexNew(&nodeStateMutexAttr);

static cz_status nodeStatus;

void canzero::init(){
	setStatus(cz_status::reset);
	// Check number of filters
	static_assert(can::filters::num_ext <= 14, "Number of CAN Ext-ID filter generated by dbc2cpp exceeds 14!");
	static_assert(can::filters::num_std <= 28, "Number of CAN Std-ID filter generated by dbc2cpp exceeds 28!");
	static_assert(can::filters::num_ext * 2 + can::filters::num_std <= 28, "Too many CAN filters configured by dbc2cpp!");
	static_assert(can::filters::num_ext != 0 || can::filters::num_std != 0, "No CAN filters were generated by dbc2cpp!");

	// Configure filters in 32-bit mode to receive Extended IDs (29-bit)
	for(int i = 0; i < can::filters::num_ext; i++) {
		CAN_FilterTypeDef sFilterConfig;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

		// A 29-bit ID consists of the 11-bit Base-ID (MSB) and 18-bit Sub-ID (LSB)
		// So STID[10:0] == EXID[28:18]

		// ID high consists of: STID[10:3] STID[2:0] EXID[17:13]  =>  so for us: id_ext[28:13]
		sFilterConfig.FilterIdHigh = ((can::filters::id_ext[i] & 0x1FFFE000ul) >> 13);

		// ID low consists of: EXID[12:5] EXID[4:0] IDE RTR 0  =>  so for us: id[23:11] 1 0 0
		sFilterConfig.FilterIdLow = ((can::filters::id_ext[i] & 0x1FFF) << 3) | 4;

		// Mask high consists of: STID[10:3] STID[2:0] EXID[17:13]  =>  so for us: mask_ext[28:13]
		sFilterConfig.FilterMaskIdHigh = ((can::filters::mask_ext[i] & 0x1FFFE000ul) >> 13);

		// Mask low consists of: EXID[12:5] EXID[4:0] IDE RTR 0  =>  so for us: mask[23:11] 1 1 0
		sFilterConfig.FilterMaskIdLow = ((can::filters::mask_ext[i] & 0x1FFF) << 3) | 6;


		// Alternate between receive FIFO0 and FIFO1 to fill both buffers.
		if ((i / 2) % 2 == 0) {
			sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		} else {
			sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
		}

		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.SlaveStartFilterBank = 14;	// Filter 0-13 for CAN1, 14-27 for CAN2.

		// Configure the filter for both CAN1 and CAN2
		sFilterConfig.FilterBank = i;
		if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
			Error_Handler(); /* Filter configuration Error */
		}

		sFilterConfig.FilterBank = 14 + i;
		if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
			Error_Handler(); /* Filter configuration Error */
		}
	}


	// Configure filters in 16-bit mode to receive Standard IDs (11-bit)
	for(int i = 0; i < can::filters::num_std; i+=2) {
		CAN_FilterTypeDef sFilterConfig;
		sFilterConfig.FilterBank = can::filters::num_ext + i / 2;	// Each STM32 filterbank has two filters in 16-bit mode, so divide by 2.
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;

		// All values are shifted by 5 bits, because the internal layout is: STDID[10:0], RTR, IDE, EXID[17:15] for a 16-bit filter.
		sFilterConfig.FilterIdHigh = can::filters::id_std[i] << 5;
		sFilterConfig.FilterMaskIdHigh = (can::filters::mask_std[i] << 5) | 0x18;  // Set bit 3 and 4, so no remote frame and 11-bit ID.

		// If number of filters is odd, configure last filter in 32-bit mode.
		if (i == can::filters::num_std - 1) {
			// Odd number of filters and last filter.
			sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

			sFilterConfig.FilterIdLow = 0;
			sFilterConfig.FilterMaskIdLow = 0x6; // Set bit 1 and 2, so ignore remote frames and only receive standard-ID frames
		} else {
			// Even number of filters or odd number and not last filter.
			sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

			sFilterConfig.FilterIdLow = can::filters::id_std[i+1] << 5;
			sFilterConfig.FilterMaskIdLow = (can::filters::mask_std[i+1] << 5) | 0x18;  // Set bit 3 and 4, so only receive standard-ID frames and ignore remote frames
		}

		// Alternate between receive FIFO0 and FIFO1 to fill both buffers.
		if ((i / 2) % 2 == 0) {
			sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		} else {
			sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
		}

		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.SlaveStartFilterBank = 14;	// Filter 0-13 for CAN1, 14-27 for CAN2.

		// Configure the filter for both CAN1 and CAN2
		if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
			Error_Handler(); /* Filter configuration Error */
		}

		sFilterConfig.FilterBank = 14 + can::filters::num_ext + i / 2;
		if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK) {
			Error_Handler(); /* Filter configuration Error */
		}
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK || HAL_CAN_Start(&hcan2) != HAL_OK) {
		Error_Handler(); /* Start Error */
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)!= HAL_OK
			|| HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING)!= HAL_OK
			|| HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR_PASSIVE)!=HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING)!= HAL_OK
			|| HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING)!= HAL_OK
			|| HAL_CAN_ActivateNotification(&hcan2, CAN_IT_ERROR_PASSIVE)!=HAL_OK) {
		Error_Handler();
	}
}

void canzero::setStatus(cz_status new_status){
	osMutexAcquire(nodeStateMutex, osWaitForever);
	nodeStatus = new_status;
	osMutexRelease(nodeStateMutex);
}


cz_status canzero::getStatus(){
	osMutexAcquire(nodeStateMutex, osWaitForever);
	cz_status returnValue = (cz_status) nodeStatus;
	osMutexRelease(nodeStateMutex);
	return returnValue;
}


