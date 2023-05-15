/*
 * AdcModuleController.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "adc.h"
#include "AdcChannel.hpp"
#include "AdcDmaController.hpp"
#include <cinttypes>
#include "estdio.hpp"
#include "AdcModule.hpp"


class AdcModuleController {
public:
	explicit AdcModuleController(AdcModule module) :
			m_hadc(AdcModuleToHandle(module)), m_nbrConvertions(
					m_hadc->Init.NbrOfConversion), m_channels(
					new AdcChannel[m_nbrConvertions]), m_buffer(
					new uint16_t[m_nbrConvertions]), m_dmaIsrId(
					AdcConvCpltCallbackHandler::getInstance().registerCallback(
							[&](ADC_HandleTypeDef *hadc) {
								if (hadc == m_hadc) {
									dmaCplrCallback();
								}
							}
					)) {
		m_semaphore = osSemaphoreNew(1, 0, NULL);
		m_mutex = osMutexNew(NULL);
		m_bussy = true;
		HAL_ADC_Start_DMA(m_hadc, reinterpret_cast<uint32_t*>(m_buffer),
				m_nbrConvertions);
		update();
	}

	~AdcModuleController() {
		AdcConvCpltCallbackHandler::getInstance().unregisterCallback(
				m_dmaIsrId);
		if (osSemaphoreGetCount(m_semaphore) != 0) {
			osSemaphoreAcquire(m_semaphore, osWaitForever);
		}
		delete[] m_channels;
		delete[] m_buffer;
	}
public:

	AdcChannel* getChannelByRank(size_t rank) {
		if (rank >= m_nbrConvertions) {
			Error_Handler();
		}
		return m_channels + rank;
	}

	void update(bool force = false) {
		if(!force && m_bussy)return;
		osMutexAcquire(m_mutex, osWaitForever);
		m_bussy = true;

		HAL_ADC_Start_DMA(m_hadc, reinterpret_cast<uint32_t*>(m_buffer),
				m_nbrConvertions);
		osSemaphoreAcquire(m_semaphore, osWaitForever);

		//update channels.
		for (size_t i = 0; i < m_nbrConvertions; i++) {
			m_channels[i].setValue(m_buffer[i]);
		}


		m_bussy = false;
		osMutexRelease(m_mutex);
	}

private:

	void dmaCplrCallback() {
		m_bussy = false;
		osSemaphoreRelease(m_semaphore);
	}

	ADC_HandleTypeDef *m_hadc;
	size_t m_nbrConvertions;
	AdcChannel *m_channels;
	uint16_t *m_buffer;
	bool m_bussy = false;
	osSemaphoreId_t m_semaphore;
	osMutexId_t m_mutex;

	unsigned int m_dmaIsrId;
};
