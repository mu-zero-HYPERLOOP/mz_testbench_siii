/*
 * AdcDmaDoubleBufferedController.hpp
 *
 *  Created on: Apr 15, 2023
 *      Author: karl
 */

#ifndef INC_ADCDMADOUBLEBUFFEREDCONTROLLER_HPP_
#define INC_ADCDMADOUBLEBUFFEREDCONTROLLER_HPP_

#include "main.h"
#include "AdcConvCpltCallbackHandler.hpp"
#include "Future.hpp"
#include "AdcResult.hpp"
#include "FreeRTOS.h"

template<typename uintx_t = uint16_t, size_t MAX_CONVERTIONS = 15>
class AdcDmaDoubleBufferedController{
public:
	explicit AdcDmaDoubleBufferedController(ADC_HandleTypeDef *hadc) :
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
		m_front = m_buffer;
		m_back = m_buffer + MAX_CONVERTIONS;
		m_frontFuture->reset();
		Future<AdcResult<uintx_t>> * frontFuture = m_frontFuture;
		HAL_ADC_Start_DMA(m_hadc, reinterpret_cast<uint32_t*>(m_front), m_nbrConvertions);
		frontFuture->get();
		frontFuture->reset();
	}

	~AdcDmaDoubleBufferedController() {
		AdcConvCpltCallbackHandler::getInstance().unregisterCallback(
				m_dmaIsrId);
	}

	Future<AdcResult<uintx_t>>* requestAdcDma() {
		//AdcResult<uintx_t> result = m_frontFuture->get();
		m_backFuture->reset();
		Future<AdcResult<uintx_t>>* frontFuture = m_frontFuture;
		HAL_ADC_Start_DMA(m_hadc, reinterpret_cast<uint32_t*>(m_back), m_nbrConvertions);
		return frontFuture;
	}

private:

	void dmaCplrCallback() {
		//Swap buffers.
		Future<AdcResult<uintx_t>>* tmp = m_frontFuture;
		m_frontFuture = m_backFuture;
		m_backFuture = tmp;
		uintx_t* tmp2 = m_front;
		m_front = m_back;
		m_back = tmp2;
		m_backFuture->complete(AdcResult<uintx_t>(m_back, m_nbrConvertions, m_resolution));
	}

	ADC_HandleTypeDef *m_hadc;
	uintx_t m_buffer[2 * MAX_CONVERTIONS];
	uintx_t* m_front;
	uintx_t* m_back;
	size_t m_nbrConvertions;
	Future<AdcResult<uintx_t>> m_futures[2];
	Future<AdcResult<uintx_t>>* m_frontFuture = &(m_futures[0]);
	Future<AdcResult<uintx_t>>* m_backFuture = &(m_futures[1]);
	unsigned int m_dmaIsrId;
	uint16_t m_resolution;

};



#endif /* INC_ADCDMADOUBLEBUFFEREDCONTROLLER_HPP_ */
