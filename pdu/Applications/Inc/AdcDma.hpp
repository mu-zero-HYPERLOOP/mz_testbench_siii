/*
 * AdcDma.hpp
 *
 *  Created on: Mar 26, 2021
 *      Author: Flo Keck
 *
 *
 *  This class reads the ADC channels configured in CubeMX using the DMA to make things faster and free CPU ressources.
 *  For example usage see TaskManager.cpp
 */

#ifndef INCLUDE_ADCDMA_HPP_
#define INCLUDE_ADCDMA_HPP_

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "estdio.hpp"


template<uint8_t NUMBER_OF_CONVERSIONS>
class AdcDma {
private:
	ADC_HandleTypeDef *m_hadc;	// Handle to the ADC

	uint8_t m_initDone = 0;
	TaskHandle_t m_xTaskToNotify = NULL;

	uint16_t m_dmaBuffer[NUMBER_OF_CONVERSIONS];	// DMA will write data here
	uint32_t m_averagingBuffer[NUMBER_OF_CONVERSIONS];	// This sums up multiple DMA readings
	float m_dataBuffer[NUMBER_OF_CONVERSIONS];	// The final data for the user

	uint16_t m_numberOfAverages = 1;
	uint16_t m_averagingCounter = 0;

public:
	AdcDma(ADC_HandleTypeDef *hadc) : m_hadc{ hadc } {
		static_assert(NUMBER_OF_CONVERSIONS <= 16, "Number of conversions needs to be less than 17!");
	}

	virtual ~AdcDma() {}

	void init(uint16_t numberOfAverages = 1) {
		m_numberOfAverages = numberOfAverages;
		if(m_numberOfAverages == 0 || m_numberOfAverages >= 1048576) {
			printf("Error in AdcDMA::init(): Number of averages invalid!\n");
			while(1);
		}

		if(NUMBER_OF_CONVERSIONS != m_hadc->Init.NbrOfConversion) {
			printf("Error in AdcDMA::init(): Number of conversions does not match the configuration in CubeMX!\n");
			while(1);
		}

		// Get task handle which should be notified later
		m_xTaskToNotify = xTaskGetCurrentTaskHandle();

		// Init done
		m_initDone = 1;
	}

	void start() {
		if(!m_initDone) {
			printf("Error in AdcDma::start(): init() was not called!\n");
			while(1);
		}

		// Zero averaging buffer
		for(int i = 0; i < NUMBER_OF_CONVERSIONS; i++) {
			m_averagingBuffer[i] = 0;
		}

		// Reset averaging counter
		m_averagingCounter = 0;

		// Start ADC in DMA mode
		HAL_ADC_Start_DMA(m_hadc, (uint32_t*)m_dmaBuffer, NUMBER_OF_CONVERSIONS);
	}

	float* getData() {
		if(!m_initDone) {
			printf("Error in AdcDma::getData(): init() was not called!\n");
			while(1);
		}

		// Divide by number of samples and return buffer
		for(int i = 0; i < NUMBER_OF_CONVERSIONS; i++) {
			m_dataBuffer[i] = m_averagingBuffer[i] / static_cast<float>(m_numberOfAverages);
		}
		return m_dataBuffer;
	}

	void ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		if(hadc == m_hadc) {
			if(!m_initDone) {
				printf("Error in AdcDma::ADC_ConvCpltCallback(): init() was not called!\n");
				while(1);
			}

			// Sum up data from DMA buffer into averaging buffer
			for(int i = 0; i < NUMBER_OF_CONVERSIONS; i++) {
				m_averagingBuffer[i] += m_dmaBuffer[i];
			}

			m_averagingCounter++;

			if(m_averagingCounter >= m_numberOfAverages) {
				// Gathered all samples, notify caller task
				vTaskNotifyGiveFromISR(m_xTaskToNotify, &xHigherPriorityTaskWoken);
			} else {
				// Need more averages, start ADC again
				HAL_ADC_Start_DMA(m_hadc, (uint32_t*)m_dmaBuffer, NUMBER_OF_CONVERSIONS);
			}
		}
		// If required yield
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
};

#endif /* INCLUDE_ADCDMA_HPP_ */
