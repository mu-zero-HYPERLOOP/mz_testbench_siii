/*
 * MultiChannelAdcSoftDmaController.hpp
 *
 *  Created on: Apr 13, 2023
 *      Author: karl
 */

#ifndef INC_ADCDMACONTROLLER_HPP_
#define INC_ADCDMACONTROLLER_HPP_

#include "main.h"
#include "AdcConvCpltCallbackHandler.hpp"
#include "Future.hpp"
#include "AdcResult.hpp"


template<typename uintx_t = uint16_t, size_t MAX_CONVERTIONS = 15>
class AdcDmaController {
public:
	explicit AdcDmaController(ADC_HandleTypeDef *hadc) :
			m_hadc(hadc), m_nbrConvertions(hadc->Init.NbrOfConversion){
		m_dmaIsrId = AdcConvCpltCallbackHandler::getInstance().registerCallback(
				[&](ADC_HandleTypeDef *hadc) {
					if (hadc == m_hadc) {
						dmaCplrCallback();
					}
				}
		);
		switch (hadc->Init.Resolution) {
		case ADC_RESOLUTION12b:
			m_resolution = (1 << 12) - 1;
			break;
		case ADC_RESOLUTION10b:
			m_resolution = (1 << 10) - 1;
			break;
		case ADC_RESOLUTION8b:
			m_resolution = (1 << 8) - 1;
			break;
		case ADC_RESOLUTION6b:
			m_resolution = (1 << 6) - 1;
			break;
		default:
			Error_Handler();
		}
	}

	~AdcDmaController() {
		AdcConvCpltCallbackHandler::getInstance().unregisterCallback(
				m_dmaIsrId);
	}

	Future<AdcResult<uintx_t>>* requestAdcDma() {
		m_future.reset();
		HAL_ADC_Start_DMA(m_hadc, reinterpret_cast<uint32_t*>(m_buffer), m_nbrConvertions);
		return &m_future;
	}

private:

	void dmaCplrCallback() {
		m_future.complete(AdcResult<uintx_t>(m_buffer, m_nbrConvertions, m_resolution));
	}

	ADC_HandleTypeDef *m_hadc;
	uintx_t m_buffer[MAX_CONVERTIONS];
	size_t m_nbrConvertions;
	Future<AdcResult<uintx_t>> m_future;
	unsigned int m_dmaIsrId;
	uint16_t m_resolution;
};

#endif /* INC_ADCDMACONTROLLER_HPP_ */
