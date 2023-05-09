/*
 * PinConfig.hpp
 *
 *  Created on: Apr 25, 2023
 *      Author: OfficeLaptop
 */

#ifndef INC_PERIPHERAL_CONFIG_HPP_
#define INC_PERIPHERAL_CONFIG_HPP_

#include "gpio.h"
#include "tim.h"
#include <cinttypes>
#include "AdcModule.hpp"

struct AdcConfig {
	AdcModule m_module;
	uint16_t m_rank;
};

struct GPIOConfig  {
	GPIO_TypeDef* m_port;
	uint16_t m_pin;
};

struct FiducialConfig{
	GPIOConfig m_gpio;
	TIM_HandleTypeDef* m_htim;
	uint32_t m_distanceBetweenInterrupts;
};

struct SDCConfig {
	GPIOConfig m_gpio;
};

struct NTCTemperaturSensorConfig {
	AdcConfig m_adc;
	float m_U0;
	float m_R;
	float m_r25;
	float m_beta;
};

struct OnBoardTemperaturConfig {
	NTCTemperaturSensorConfig m_internalNTCConfig;
	NTCTemperaturSensorConfig m_externalNTCConfig;
	AdcConfig m_inputVoltageConfig;
};

struct PressureConfig {
	AdcConfig m_adc;
};

struct PeripheralConfig {
	FiducialConfig m_fiducialRightConfig;
	FiducialConfig m_fiducialLeftConfig;
	SDCConfig m_sdcConfig;
	NTCTemperaturSensorConfig m_coolingReservoirTemperaturSensorConfig;
	NTCTemperaturSensorConfig m_eboxTemperaturConfig;
	OnBoardTemperaturConfig m_onBoardTemperaturConfig;
	PressureConfig m_pressureConfig;
};

constexpr PeripheralConfig g_peripherals = {
	.m_fiducialRightConfig = {
			.m_gpio = {
				.m_port = DIN2_GPIO_Port,
				.m_pin = DIN2_Pin,
			},
			.m_htim = &htim3,
			.m_distanceBetweenInterrupts = 80,
	},
	.m_fiducialLeftConfig = {
			.m_gpio = {
				.m_port = DIN1_GPIO_Port,
				.m_pin = DIN1_Pin,
			},
			.m_htim = &htim9,
			.m_distanceBetweenInterrupts = 80,
	},
	.m_sdcConfig = {
			.m_gpio = {
				.m_port = SDC_GPIO_Port,
				.m_pin = SDC_Pin,
			},
	},
	.m_coolingReservoirTemperaturSensorConfig = {
			.m_adc = {
				.m_module = ADC_MODULE2,
				.m_rank = 2,
			},
			.m_U0 = 0,
			.m_R = 0,
			.m_r25 = 0,
			.m_beta = 0,
	},
	.m_eboxTemperaturConfig = {
				.m_adc = {
					.m_module = ADC_MODULE2,
					.m_rank = 0, //TODO fix rank.
				},
				//TODO set values correctly.
				.m_U0 = 0,
				.m_R = 0,
				.m_r25 = 0,
				.m_beta = 0,
	},
	.m_onBoardTemperaturConfig = {
			.m_internalNTCConfig = {
				.m_adc = {
					.m_module = ADC_MODULE1,
					.m_rank = 2,
				},
				.m_U0 = 0,
				.m_R = 0,
				.m_r25 = 0,
				.m_beta = 0,
			},
			.m_externalNTCConfig = {
				.m_adc = {
					.m_module = ADC_MODULE1,
					.m_rank = 0,
				},
				.m_U0 = 0,
				.m_R = 0,
				.m_r25 = 0,
				.m_beta = 0,
			},
			.m_inputVoltageConfig = {
				.m_module = ADC_MODULE1	,
				.m_rank = 1
			},
	},
	.m_pressureConfig = {
		.m_adc {
			.m_module = ADC_MODULE2,
			.m_rank = 2 //TODO fix rank.
		}
	},
};



#endif /* INC_PERIPHERAL_CONFIG_HPP_ */
