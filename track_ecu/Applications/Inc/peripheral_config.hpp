/*
 * peripheral_config.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "AdcModule.hpp"
#include <cinttypes>
#include "gpio.h"

struct AdcConfig {
	AdcModule m_module;
	uint16_t m_rank;
};

struct GPIOConfig  {
	GPIO_TypeDef* m_port;
	uint16_t m_pin;
};


struct DistanceSensorConfig {
	AdcConfig m_adc;
	//TODO add more configuration options.
};

struct PneumaticPistonControllerConfig {
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

struct PeripheralConfig {
	OnBoardTemperaturConfig m_onBoardTemperaturConfig;
	DistanceSensorConfig m_distanceSensor;
	PneumaticPistonControllerConfig m_pneumaticPistonConfig;

};

constexpr PeripheralConfig  g_peripherals = {
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
		.m_distanceSensor = {
				.m_adc = {
					.m_module = ADC_MODULE2,
					.m_rank = 0 //TODO fix rank.
				},
		},
		.m_pneumaticPistonConfig = {
			.m_gpio = {
				.m_port = DIN1_GPIO_Port, //TODO update pin assignment
				.m_pin = DIN1_Pin
			},
		},
};
