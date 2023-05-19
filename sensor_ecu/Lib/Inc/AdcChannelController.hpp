/*
 * AdcChannelController.hpp
 *
 *  Created on: Apr 26, 2023
 *      Author: OfficeLaptop
 */
#pragma once

#include "AdcModuleController.hpp"
#include "AdcChannel.hpp"

template<AdcModule ADC_MODULE>
static AdcModuleController& _getChannelAdcModuleControllerCompiletime(){
	static AdcModuleController instance(ADC_MODULE);
	return instance;
}

static AdcModuleController& _getChannelAdcModuleControllerRuntime(AdcModule module){
	switch(module){
	case ADC_MODULE1: return _getChannelAdcModuleControllerCompiletime<ADC_MODULE1>();
	case ADC_MODULE2: return _getChannelAdcModuleControllerCompiletime<ADC_MODULE2>();
	default:
		Error_Handler();
		return _getChannelAdcModuleControllerCompiletime<ADC_MODULE1>();
	}
}



class AdcChannelController {
public:
	explicit AdcChannelController(AdcModule module, unsigned int rank) :
			m_module(&_getChannelAdcModuleControllerRuntime(module)),
			m_channel(m_module->getChannelByRank(rank)) {
	}
	AdcChannelController() : m_module(nullptr), m_channel(nullptr){

	}

	uint16_t get(bool force = false) {
		if(m_module == nullptr || m_channel == nullptr){
			printf("ERROR: can't fetch value from uninitalized adc channel controller!\n");
			Error_Handler();
		}
		m_module->update(force);
		return m_channel->get();
	}
private:
	AdcModuleController* m_module;
	AdcChannel *m_channel;
};
