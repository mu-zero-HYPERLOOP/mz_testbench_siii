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
#include "FreeRTOS.h"
#include "cmsis_os.h"

class AdcModuleController {
public:
	explicit AdcModuleController(AdcModule module) :
			m_hadc(AdcModuleToHandle(module)), m_nbrConvertions(
					m_hadc->Init.NbrOfConversion), m_channels(
					new AdcChannel[m_nbrConvertions]), m_dmaIsrId(
					AdcConvCpltCallbackHandler::getInstance().registerCallback(
							[&](ADC_HandleTypeDef *hadc) {
								if (hadc == m_hadc) {
									dmaCplrCallback();
								}
							}
					)) {
		if(ENABLE_SOFTWARE_AVERAGE){
			m_averageBuffer = new float[m_nbrConvertions];
			m_buffer = new uint16_t[m_nbrConvertions * SOFTWARE_AVERAGE_SAMPLE_COUNT];
		}else{
			m_buffer = new uint16_t[m_nbrConvertions];
		}
		m_mutex = osMutexNew(NULL);
		update(true);
	}

	~AdcModuleController() {
		AdcConvCpltCallbackHandler::getInstance().unregisterCallback(
				m_dmaIsrId);
		osMutexDelete(m_mutex);
		if(ENABLE_SOFTWARE_AVERAGE){
			delete[] m_averageBuffer;
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
		osMutexAcquire(m_mutex, osWaitForever);
		TickType_t timeSinceLastConvertion = xTaskGetTickCount()
				- lastConvertionTime;
		if (force || timeSinceLastConvertion > MIN_TIME_BETWEEN_CONVERTIONS) {
			lastConvertionTime = xTaskGetTickCount();
			if (ENABLE_SOFTWARE_AVERAGE) {
				for(size_t i=0;i<m_nbrConvertions;i++){
					m_averageBuffer[i] = 0;
				}
				HAL_ADC_Start_DMA(m_hadc, reinterpret_cast<uint32_t*>(m_buffer),
						m_nbrConvertions * SOFTWARE_AVERAGE_SAMPLE_COUNT);
			} else {
				HAL_ADC_Start_DMA(m_hadc, reinterpret_cast<uint32_t*>(m_buffer),
						m_nbrConvertions);
			}
		}
		osMutexRelease(m_mutex);
	}

private:

	void dmaCplrCallback() {
		//update channels.
		if (ENABLE_SOFTWARE_AVERAGE) {
			for(size_t idx = 0, i = 0; i < SOFTWARE_AVERAGE_SAMPLE_COUNT; i++){
				for (size_t j = 0; j < m_nbrConvertions; j++, idx++) {
					float avalue = static_cast<float>(m_buffer[idx]);
					m_averageBuffer[j] += avalue;
				}
			}
			for(size_t i=0; i < m_nbrConvertions; i++){
				m_channels[i].setValue(m_averageBuffer[i] / SOFTWARE_AVERAGE_SAMPLE_COUNT);
			}
		} else {
			for (size_t i = 0; i < m_nbrConvertions; i++) {
				m_channels[i].setValue(m_buffer[i]);
			}
		}
	}

	ADC_HandleTypeDef *m_hadc;
	size_t m_nbrConvertions;
	AdcChannel *m_channels;
	uint16_t *m_buffer;
	float* m_averageBuffer;
	osMutexId_t m_mutex;
	unsigned int m_dmaIsrId;
	TickType_t lastConvertionTime = 0;
	static constexpr TickType_t MIN_TIME_BETWEEN_CONVERTIONS = pdMS_TO_TICKS(5);
	static constexpr bool ENABLE_SOFTWARE_AVERAGE = true;
	static constexpr size_t SOFTWARE_AVERAGE_SAMPLE_COUNT = 4;
};
