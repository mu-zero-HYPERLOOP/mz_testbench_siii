/*
 * NewPDU.hpp
 *
 *  Created on: May 8, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include "led.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "canzero.hpp"
#include "tim.h"
#include "Watchdog.hpp"
#include <cinttypes>
#include "AdcChannelController.hpp"
#include "MovingAverageFilter.hpp"

class NewPDU {
private:

public:

	void start() {
		for (size_t i = 0; i < 10; i++) {
			m_lpChannelAdc[i] = AdcChannelController(ADC_MODULE2, i);
			for (size_t j = 0; j < 10; j++) {
				float v = m_lpChannelAdc[i].get();
				m_lpAverage[i].update(v);
			}
		}
		for (size_t i = 0; i < 4; i++) {
			m_hpChannelAdc[i] = AdcChannelController(ADC_MODULE2, 10 + i);
			for (size_t j = 0; j < 10; j++) {
				float v = m_hpChannelAdc[i].get();
				m_lpAverage[i].update(v);
			}
		}

		// Start timers for PWM generation
		HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);	// LPCh1
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// LPCh2
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// LPCh3
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);	// LPCh8
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);	// LPCh9
		HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);	// LPCh10
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);	// HPCh1
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);	// HPCh2
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	// D1
		HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);	// D2
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);	// D3
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	// D4

		m_rxControllQueue.enable();
		m_lpDutycycleQueue.enable();
		m_hpDutycycleQueue.enable();
		m_manualControllQueue.enable();

		while (true) {

			// Receive control messages
			receiveCanMessages();

			if (xTaskGetTickCount() - m_lastDataRead
					>= OD_currentReadInterval_get()) {
				m_lastDataRead = xTaskGetTickCount();

				// Read the current of all channels and send it
				readAdcChannels();
				sendHpCurrentCAN();
				osDelay(1);
				sendLpCurrentCAN();
				osDelay(1);
				sendShortCircuitDebugCAN();
			}

			if (m_enabled && m_stateMaschineWatchdog.isTimedOut()) {
				ERR_watchdogStateMachine_set();
			}

			if (m_outputState.SDC.get()) {
				LED_Orange_Write(255);
			} else {
				LED_Orange_Write(0);
			}

			// Send out status message with configured interval or when there is a change in PDU status
			if (xTaskGetTickCount() - m_lastStatusSent
					>= OD_statusSendInterval_get()
					|| m_lastPduEnabled != m_enabled
					|| m_lastPeHwEnabled != m_outputState.LPCh6.get()
					|| m_lastErrorFlag != anyErrorPresent()) {

				m_lastPduEnabled = m_enabled;
				m_lastPeHwEnabled = m_outputState.LPCh6.get();
				m_lastErrorFlag = anyErrorPresent();

				m_lastStatusSent = xTaskGetTickCount();

				// Send out data to CAN bus
				sendStatusCAN();
			}

			// Delay until next check
			osDelay(pdMS_TO_TICKS(1000));
		}
		return;

	}

private:

	void readAdcChannels() {
		float lvCurrent = 0;
		for (int i = 0; i < 10; i++) { //maybe the values for i needs to be changed because the ADC channels have changed.
			// Raw ADC value is converted into a voltage
			// The sense current is calculated through the 62Ohms current sense resistor
			// The sense current is multiplied by the current divider factor of BTF6070

			float current = (m_lpChannelAdc[i].get() * 3.3f / 4095.0f) / 62.0f
					* 1750 - 0.010f;// Subtract 10mA since this seems to be an offset

			// Limit to plausible values
			if (current < 0.0f) {
				current = 0.0f;
			}
			if (current > 8.0f) {
				current = 8.0f;
			}

			// Sum up current to total LV battery current
			if (current < 8.0f) {
				// Do not sum up error current values
				lvCurrent += current;
			}
			m_lpAverage[i].update(current);
		}

		for (int i = 0; i < 4; i++) {
			// Raw ADC value is converted into a voltage
			// The sense current is calculated through the 62Ohms current sense resistor
			// The sense current is multiplied by the current divider factor of BTT6010
			float current = (m_hpChannelAdc[i].get() * 3.3f / 4095.0f) / 62.0f
					* 4000 - 0.300f;// Subtract 300mA since this seems to be an offset

			// Limit to plausible values
			if (current < 0.0f) {
				current = 0.0f;
			}
			if (current > 16.0f) {
				current = 16.0f;
			}

			// Sum up current to total LV battery current
			if (current < 16.0f) {
				// Do not sum up error current values
				lvCurrent += current;
			}
			m_hpAverage[i].update(current);
		}

		//clamp to [0,40]
		if (lvCurrent > 40) {
			lvCurrent = 40;
		}
		if (lvCurrent < 0) {
			lvCurrent = 0;
		}

		m_outputState.LPCh1_opticalSensor.setCurrent(m_lpAverage[0].get());
		m_outputState.LPCh2.setCurrent(m_lpAverage[1].get());
		m_outputState.LPCh3_HVCU.setCurrent(m_lpAverage[2].get());
		m_outputState.LPCh4_frontECU.setCurrent(m_lpAverage[3].get());
		m_outputState.LPCh5_powerElectronics.setCurrent(m_lpAverage[4].get());
		m_outputState.LPCh6.setCurrent(m_lpAverage[5].get());
		m_outputState.LPCh7_rearECU.setCurrent(m_lpAverage[6].get());
		m_outputState.LPCh8_telemetry.setCurrent(m_lpAverage[7].get());
		m_outputState.LPCh9_logger.setCurrent(m_lpAverage[8].get());
		m_outputState.LPCh10.setCurrent(m_lpAverage[9].get());

		m_outputState.HPCh1_projectXX.setCurrent(m_hpAverage[0].get());
		m_outputState.HPCh2_coolingPump.setCurrent(m_hpAverage[1].get());
		m_outputState.HPCh3.setCurrent(m_hpAverage[2].get());
		m_outputState.HPCh4.setCurrent(m_hpAverage[3].get());
	}

	void sendLpCurrentCAN() {
		// Current of LPchannels 1 to 5
		can::Message<can::messages::PDU_TX_LP_Current1> msgCurrent1;
		msgCurrent1.set<can::signals::PDU_LPCh1_Current>(m_lpAverage[0].get());
		msgCurrent1.set<can::signals::PDU_LPCh2_Current>(m_lpAverage[1].get());
		msgCurrent1.set<can::signals::PDU_LPCh3_Current>(m_lpAverage[2].get());
		msgCurrent1.set<can::signals::PDU_LPCh4_Current>(m_lpAverage[3].get());
		msgCurrent1.set<can::signals::PDU_LPCh5_Current>(m_lpAverage[4].get());
		msgCurrent1.send();

	}

	void sendHpCurrentCAN() {
		// Current of HPchannels 1 to 4
		can::Message<can::messages::PDU_TX_HP_Current> msgCurrent3;
		msgCurrent3.set<can::signals::PDU_HPCh1_Current>(m_hpAverage[0].get());
		msgCurrent3.set<can::signals::PDU_HPCh2_Current>(m_hpAverage[1].get());
		msgCurrent3.set<can::signals::PDU_HPCh3_Current>(m_hpAverage[2].get());
		msgCurrent3.set<can::signals::PDU_HPCh4_Current>(m_hpAverage[3].get());
		msgCurrent3.send();
	}

	void sendShortCircuitDebugCAN() {

		// Short Circuit Debug Message
		can::Message<can::messages::PDU_TX_LP_Short_Circuit_Debug> msgDebug1;
		msgDebug1.set<can::signals::PDU_LPCh1_ShortCnt>(
				m_outputState.LPCh1_opticalSensor.numShorts());
		msgDebug1.set<can::signals::PDU_LPCh1_State>(
				m_outputState.LPCh1_opticalSensor.getStatus());
		msgDebug1.set<can::signals::PDU_LPCh2_ShortCnt>(
				m_outputState.LPCh2.numShorts());
		msgDebug1.set<can::signals::PDU_LPCh2_State>(
				m_outputState.LPCh2.getStatus());
		msgDebug1.set<can::signals::PDU_LPCh3_ShortCnt>(
				m_outputState.LPCh3_HVCU.numShorts());
		msgDebug1.set<can::signals::PDU_LPCh3_State>(
				m_outputState.LPCh3_HVCU.getStatus());
		msgDebug1.set<can::signals::PDU_LPCh4_ShortCnt>(
				m_outputState.LPCh4_frontECU.numShorts());
		msgDebug1.set<can::signals::PDU_LPCh4_State>(
				m_outputState.LPCh4_frontECU.getStatus());
		msgDebug1.set<can::signals::PDU_LPCh5_ShortCnt>(
				m_outputState.LPCh5_powerElectronics.numShorts());
		msgDebug1.set<can::signals::PDU_LPCh5_State>(
				m_outputState.LPCh5_powerElectronics.getStatus());
		msgDebug1.set<can::signals::PDU_LPCh6_ShortCnt>(
				m_outputState.LPCh6.numShorts());
		msgDebug1.set<can::signals::PDU_LPCh6_State>(
				m_outputState.LPCh6.getStatus());
		msgDebug1.set<can::signals::PDU_LPCh7_ShortCnt>(
				m_outputState.LPCh7_rearECU.numShorts());
		msgDebug1.set<can::signals::PDU_LPCh7_State>(
				m_outputState.LPCh7_rearECU.getStatus());
		msgDebug1.set<can::signals::PDU_LPCh8_ShortCnt>(
				m_outputState.LPCh8_telemetry.numShorts());
		msgDebug1.set<can::signals::PDU_LPCh8_State>(
				m_outputState.LPCh8_telemetry.getStatus());
		msgDebug1.set<can::signals::PDU_LPCh9_ShortCnt>(
				m_outputState.LPCh9_logger.numShorts());
		msgDebug1.set<can::signals::PDU_LPCh9_State>(
				m_outputState.LPCh9_logger.getStatus());
		msgDebug1.set<can::signals::PDU_LPCh10_ShortCnt>(
				m_outputState.LPCh10.numShorts());
		msgDebug1.set<can::signals::PDU_LPCh10_State>(
				m_outputState.LPCh10.getStatus());
		msgDebug1.send();

		can::Message<can::messages::PDU_TX_HP_Short_Circuit_Debug> msgDebug2;
		msgDebug2.set<can::signals::PDU_HPCh1_ShortCnt>(
				m_outputState.HPCh1_projectXX.numShorts());
		msgDebug2.set<can::signals::PDU_HPCh1_State>(
				m_outputState.HPCh1_projectXX.getStatus());
		msgDebug2.set<can::signals::PDU_HPCh2_ShortCnt>(
				m_outputState.HPCh2_coolingPump.numShorts());
		msgDebug2.set<can::signals::PDU_HPCh2_State>(
				m_outputState.HPCh2_coolingPump.getStatus());
		msgDebug2.set<can::signals::PDU_HPCh3_ShortCnt>(
				m_outputState.HPCh3.numShorts());
		msgDebug2.set<can::signals::PDU_HPCh3_State>(
				m_outputState.HPCh3.getStatus());
		msgDebug2.set<can::signals::PDU_HPCh4_ShortCnt>(
				m_outputState.HPCh4.numShorts());
		msgDebug2.set<can::signals::PDU_HPCh4_State>(
				m_outputState.HPCh4.getStatus());
		msgDebug2.send();
	}

	void receiveCanMessages() {
		while (m_rxControllQueue.hasAny()) {
			// Reset watchdog for state machine
			m_stateMaschineWatchdog.reset();
			can::Message<can::messages::PDU_RX_Control_NoOD> controlMsg =
					m_rxControllQueue.dequeue();
			bool errorReset = controlMsg.get<can::signals::PDU_RX_ErrorReset>();
			bool _enable = controlMsg.get<can::signals::PDU_RX_Enable>(); //activates control mode
			bool peHwEnable = controlMsg.get<can::signals::PDU_RX_PEHWEnable>(); //allows the PDU to set the PE_enable channel (D2)

			// Reset all errors if requested
			if (errorReset) {
				resetAllErrors();
			}

			// Edge detection: Mode change from Manual Control to State Machine Control
			if (!m_enabled && _enable) { // Enable was just set
				if (!anyErrorPresent()) { // Do not enable if there is still an error present
					m_enabled = _enable;

					// Reset all output channels to their default values:
					m_outputState = { };
				}

				// Set CANzero status to operational -> changing of critical OD entries now locked
				canzero::setStatus(operational);
			}

			// Edge detection: Mode change from State Machine Control to Manual Control
			if (m_enabled && !_enable) { // Enable was reset
				m_enabled = _enable;

				// Reset all output channels to their default values:
				m_outputState = { };

				// Set CANzero status to default state
				canzero::setStatus(pre_operational);

				// Let cooling pump run until one minute after disabling
				m_disableTime = xTaskGetTickCount();
			}

			// PDU is pduEnabled and in State Machine control mode
			if (m_enabled) {
				m_outputState.SDC.set(true);
				m_outputState.LPCh5_powerElectronics.set(true);
				m_outputState.LPCh6.set(peHwEnable);
			}
		}

		while (m_manualControllQueue.hasAny()) {
			printf("manuel controll\n");
			can::Message<can::messages::PDU_RX_Manual_Control> manualControlMsg =
					m_manualControllQueue.dequeue();

			// PDU is in manual control mode
			if (!m_enabled) {
				m_outputState.LPCh1_opticalSensor.set(
						manualControlMsg.get<can::signals::PDU_LPCh1_Enable>());
				m_outputState.LPCh2.set(
						manualControlMsg.get<can::signals::PDU_LPCh2_Enable>());
				m_outputState.LPCh3_HVCU.set(
						manualControlMsg.get<can::signals::PDU_LPCh3_Enable>());
				m_outputState.LPCh4_frontECU.set(
						manualControlMsg.get<can::signals::PDU_LPCh4_Enable>());
				m_outputState.LPCh5_powerElectronics.set(
						manualControlMsg.get<can::signals::PDU_LPCh5_Enable>());
				m_outputState.LPCh6.set(
						manualControlMsg.get<can::signals::PDU_LPCh6_Enable>());
				m_outputState.LPCh7_rearECU.set(
						manualControlMsg.get<can::signals::PDU_LPCh7_Enable>());
				m_outputState.LPCh8_telemetry.set(
						manualControlMsg.get<can::signals::PDU_LPCh8_Enable>());
				m_outputState.LPCh9_logger.set(
						manualControlMsg.get<can::signals::PDU_LPCh9_Enable>());
				m_outputState.LPCh10.set(
						manualControlMsg.get<can::signals::PDU_LPCh10_Enable>());

				m_outputState.HPCh1_projectXX.set(
						manualControlMsg.get<can::signals::PDU_HPCh1_Enable>());
				m_outputState.HPCh2_coolingPump.set(
						manualControlMsg.get<can::signals::PDU_HPCh2_Enable>());
				m_outputState.HPCh3.set(
						manualControlMsg.get<can::signals::PDU_HPCh3_Enable>());
				m_outputState.HPCh4.set(
						manualControlMsg.get<can::signals::PDU_HPCh4_Enable>());

				m_outputState.D1_projectXX.set(
						manualControlMsg.get<can::signals::PDU_D1_Enable>());
				m_outputState.D2_PE_enable.set(
						manualControlMsg.get<can::signals::PDU_D2_Enable>());
				m_outputState.D3.set(
						manualControlMsg.get<can::signals::PDU_D3_Enable>());
				m_outputState.D4.set(
						manualControlMsg.get<can::signals::PDU_D4_Enable>());

				m_outputState.SDC.set(
						manualControlMsg.get<can::signals::PDU_SDC_Enable>());
			}
		}

		while (m_lpDutycycleQueue.hasAny()) {
			printf("lp duty\n");
			can::Message<can::messages::PDU_RX_LP_Dutycycle_NoOD> dutyMsg =
					m_lpDutycycleQueue.dequeue();
			// Update duty cycles when PDU is in manual control mode
			if (!m_enabled) {
				m_outputState.LPCh1_opticalSensor.setDuty(
						dutyMsg.get<can::signals::PDU_LPCh1_Dutycycle>());
				m_outputState.LPCh2.setDuty(
						dutyMsg.get<can::signals::PDU_LPCh2_Dutycycle>());
				m_outputState.LPCh3_HVCU.setDuty(
						dutyMsg.get<can::signals::PDU_LPCh3_Dutycycle>());
				m_outputState.LPCh8_telemetry.setDuty(
						dutyMsg.get<can::signals::PDU_LPCh8_Dutycycle>());
				m_outputState.LPCh9_logger.setDuty(
						dutyMsg.get<can::signals::PDU_LPCh9_Dutycycle>());
				m_outputState.LPCh10.setDuty(
						dutyMsg.get<can::signals::PDU_LPCh10_Dutycycle>());
			}
		}

		while (m_hpDutycycleQueue.hasAny()) {
			printf("hp duty\n");
			can::Message<can::messages::PDU_RX_HP_D_Dutycycle_NoOD> dutyMsg =
					m_hpDutycycleQueue.dequeue();
			if (!m_enabled) {
				m_outputState.HPCh1_projectXX.setDuty(
						dutyMsg.get<can::signals::PDU_HPCh1_Dutycycle>());
				m_outputState.HPCh2_coolingPump.setDuty(
						dutyMsg.get<can::signals::PDU_HPCh2_Dutycycle>());
				m_outputState.D1_projectXX.setDuty(
						dutyMsg.get<can::signals::PDU_D1_Dutycycle>());
				m_outputState.D2_PE_enable.setDuty(
						dutyMsg.get<can::signals::PDU_D2_Dutycycle>());
				m_outputState.D3.setDuty(
						dutyMsg.get<can::signals::PDU_D3_Dutycycle>());
				m_outputState.D4.setDuty(
						dutyMsg.get<can::signals::PDU_D4_Dutycycle>());
			}
		}
	}

	void updateChannels() {
		m_outputState.LPCh1_opticalSensor.update();
		m_outputState.LPCh2.update();
		m_outputState.LPCh3_HVCU.update();
		m_outputState.LPCh4_frontECU.update();
		m_outputState.LPCh5_powerElectronics.update();
		m_outputState.LPCh6.update();
		m_outputState.LPCh7_rearECU.update();
		m_outputState.LPCh8_telemetry.update();
		m_outputState.LPCh9_logger.update();
		m_outputState.LPCh10.update();
		m_outputState.HPCh1_projectXX.update();
		m_outputState.HPCh2_coolingPump.update();
		m_outputState.HPCh3.update();
		m_outputState.HPCh4.update();
		m_outputState.D1_projectXX.update();
		m_outputState.D2_PE_enable.update();
		m_outputState.D3.update();
		m_outputState.D4.update();

		// Standard output channels with PWM support

		//Somehow all timers except of TIM2 can't save dutycycles of 100% so that the +1 was deleted.

		// LPCh1 is TIM12_CH2
		if (m_outputState.LPCh1_opticalSensor.getSwitch()) {
			htim12.Instance->CCR2 = m_outputState.LPCh1_opticalSensor.getDuty()
					* (htim12.Instance->ARR) / 100.0f;
		} else {
			htim12.Instance->CCR2 = 0;
		}

		// LPCh2 is TIM2_CH3
		if (m_outputState.LPCh2.getSwitch()) {
			htim2.Instance->CCR3 = m_outputState.LPCh2.getDuty()
					* (htim2.Instance->ARR + 1) / 100.0f;
		} else {
			htim2.Instance->CCR3 = 0;
		}

		// LPCh3 is TIM2_CH1
		if (m_outputState.LPCh3_HVCU.getSwitch()) {
			htim2.Instance->CCR1 = m_outputState.LPCh3_HVCU.getDuty()
					* (htim2.Instance->ARR + 1) / 100.0f;
		} else {
			htim2.Instance->CCR1 = 0;
		}

		// LPCh8 is TIM8_CH1
		if (m_outputState.LPCh8_telemetry.getSwitch()) {
			htim8.Instance->CCR1 = m_outputState.LPCh3_HVCU.getDuty()
					* (htim8.Instance->ARR) / 100.0f;
		} else {
			htim8.Instance->CCR1 = 0;
		}

		// LPCh9 is TIM4_CH2
		if (m_outputState.LPCh9_logger.getSwitch()) {
			htim4.Instance->CCR2 = m_outputState.LPCh9_logger.getDuty()
					* (htim4.Instance->ARR) / 100.0f;
		} else {
			htim4.Instance->CCR2 = 0;
		}

		// LPCh10 is TIM11_CH1
		if (m_outputState.LPCh10.getSwitch()) {
			htim11.Instance->CCR1 = m_outputState.LPCh10.getDuty()
					* (htim11.Instance->ARR) / 100.0f;
		} else {
			htim11.Instance->CCR1 = 0;
		}

		// HPCh1 is TIM8_CH4
		if (m_outputState.HPCh1_projectXX.getSwitch()) {
			htim8.Instance->CCR4 = m_outputState.HPCh1_projectXX.getDuty()
					* (htim8.Instance->ARR) / 100.0f;
		} else {
			htim8.Instance->CCR4 = 0;
		}

		// HPCh2 is TIM8_CH2
		if (m_outputState.HPCh2_coolingPump.getSwitch()) {
			htim8.Instance->CCR2 = m_outputState.HPCh2_coolingPump.getDuty()
					* (htim8.Instance->ARR) / 100.0f;
		} else {
			htim8.Instance->CCR2 = 0;
		}

		// D1 is TIM3_CH1 (is controlled by ProjectXX.hpp)
		if (m_outputState.D1_projectXX.getSwitch()) {
			//htim3.Instance->CCR1 = outputState.D1_projectXX.getDuty() * (htim3.Instance->ARR) / 100.0f;
		} else {
			//htim3.Instance->CCR1= 0;
		}

		// D2 is TIM10_CH1
		if (m_outputState.D2_PE_enable.getSwitch()) {
			htim10.Instance->CCR1 = m_outputState.D2_PE_enable.getDuty()
					* (htim10.Instance->ARR) / 100.0f;
		} else {
			htim10.Instance->CCR1 = 0;
		}

		// D3 is TIM2_CH4
		if (m_outputState.D3.getSwitch()) {
			htim2.Instance->CCR4 = m_outputState.D3.getDuty()
					* (htim2.Instance->ARR) / 100.0f;
		} else {
			htim2.Instance->CCR4 = 0;
		}

		// D4 is TIM3_CH3
		if (m_outputState.D4.getSwitch()) {
			htim3.Instance->CCR3 = m_outputState.D4.getDuty()
					* (htim3.Instance->ARR) / 100.0f;
		} else {
			htim3.Instance->CCR3 = 0;
		}

		// Standard On/Off output
		HAL_GPIO_WritePin(LP4_control_GPIO_Port, LP4_control_Pin,
				m_outputState.LPCh4_frontECU.getSwitch() ?
						GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LP5_control_GPIO_Port, LP5_control_Pin,
				m_outputState.LPCh5_powerElectronics.getSwitch() ?
						GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LP6_control_GPIO_Port, LP6_control_Pin,
				m_outputState.LPCh6.getSwitch() ?
						GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LP7_control_GPIO_Port, LP7_control_Pin,
				m_outputState.LPCh7_rearECU.getSwitch() ?
						GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HP3_control_GPIO_Port, HP3_control_Pin,
				m_outputState.HPCh3.getSwitch() ?
						GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HP4_control_GPIO_Port, HP4_control_Pin,
				m_outputState.HPCh4.getSwitch() ?
						GPIO_PIN_SET : GPIO_PIN_RESET);

		// SDC switch
		HAL_GPIO_WritePin(SDC_control_GPIO_Port, SDC_control_Pin,
				m_outputState.SDC.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

	void sendStatusCAN() {
		// PDU status
		can::Message<can::messages::PDU_TX_Status> msgStatus;
		msgStatus.set<can::signals::PDU_TX_Enabled>(m_enabled);
		msgStatus.set<can::signals::PDU_TX_PEHWEnabled>(
				m_outputState.LPCh6.get());
		msgStatus.set<can::signals::PDU_TX_ErrorFlag>(anyErrorPresent());
		msgStatus.send();
	}

	/**
	 * Checks if any error is present
	 * @return true if any error is present
	 */
	bool anyErrorPresent() {
		return ERR_watchdogStateMachine_get();
	}

	/**
	 * Reset all errors
	 */
	void resetAllErrors() {
		ERR_batterVoltageCritical_clear();
		ERR_batteryOvercurrent_clear();
		ERR_watchdogStateMachine_clear();
	}

private:
	class OutputChannelPwm {
	private:
		static constexpr uint32_t m_waitingPeriodAfterChange = 125;
		static constexpr float m_errorCurrent = 7.0f;
		static constexpr uint8_t m_shortCircuitResetAttempts = 14;

		static_assert(m_shortCircuitResetAttempts < 15, "Has to be smaller than 15!");

		float m_current;
		bool m_on = false;
		float m_duty = 0.0f;
		bool m_switchOn = false;
		uint32_t m_lastChange = 0;
		uint8_t m_shortCircuitsDetected = 0;
		bool m_shortCircuitResetOngoing = false;

		using OUTPUT_STATE = can::signals::PDU_LPCh1_State;

	public:
		OutputChannelPwm(bool _on = false, float _duty = 100.0f) :
				m_on { _on }, m_duty { _duty }, m_switchOn { _on } {
			m_lastChange = xTaskGetTickCount();
		}

		void set(bool _on, float _duty = -100.0f) {
			if (_on != m_on || (_duty >= 0.0f && _duty != m_duty)) {
				m_lastChange = xTaskGetTickCount();
				m_on = _on;
				if (m_duty >= 0.0f) {
					m_duty = _duty;
				}
				m_switchOn = m_on;

				if (!m_on) {
					m_shortCircuitsDetected = 0;
				}
			}
		}
		void setDuty(float _duty) {
			m_duty = _duty;
		}
		float getDuty() const {
			return m_duty;
		}
		bool get() const {
			return m_on;
		}

		bool getSwitch() const {
			return m_switchOn;
		}

		void setCurrent(float _current) {
			m_current = _current;
		}

		uint8_t getStatus() {
			if (xTaskGetTickCount() - m_lastChange
					>= m_waitingPeriodAfterChange) {
				if (m_current >= m_errorCurrent) {
					if (m_on) {
						return OUTPUT_STATE::OUTPUT_SHORT_CIRCUIT;
					} else {
						return OUTPUT_STATE::EXTERNAL_VOLTAGE;
					}
				} else {
					return OUTPUT_STATE::OK;
				}
			} else {
				return OUTPUT_STATE::STATUS_CHANGE_PENDING;
			}
		}

		uint8_t numShorts() {
			return m_shortCircuitsDetected;
		}

		void update() {
			uint8_t status = getStatus();

			if (status == OUTPUT_STATE::OUTPUT_SHORT_CIRCUIT) {
				if (m_shortCircuitsDetected <= m_shortCircuitResetAttempts) {
					m_shortCircuitsDetected++;

					m_switchOn = false;
					m_lastChange = xTaskGetTickCount();
					m_shortCircuitResetOngoing = true;
				}
			}

			status = getStatus();

			if (m_shortCircuitResetOngoing
					&& xTaskGetTickCount() - m_lastChange >= 20) { //  status != OUTPUT_STATE::STATUS_CHANGE_PENDING
				m_switchOn = true;
				m_lastChange = xTaskGetTickCount();
				m_shortCircuitResetOngoing = false;
			}
		}

	};

	class OutputChannel: public OutputChannelPwm {
	public:
		OutputChannel(bool _on = false) :
				OutputChannelPwm { _on, 100.0f } {
		}
	};

// All output channels of the PDU with the initial states
	struct PduOutputState {
		OutputChannelPwm LPCh1_opticalSensor { true, 100.0f };
		OutputChannelPwm LPCh2 { true, 100.0f };
		OutputChannelPwm LPCh3_HVCU { true, 100.0f };
		OutputChannel LPCh4_frontECU { true };
		OutputChannel LPCh5_powerElectronics { true };
		OutputChannel LPCh6 { false }; //hw
		OutputChannel LPCh7_rearECU { true };
		OutputChannelPwm LPCh8_telemetry { true, 100.0f }; // Telemetry node, Wifi-Router
		OutputChannelPwm LPCh9_logger { true };
		OutputChannelPwm LPCh10 { true, 100.0f }; //telemetry
		OutputChannelPwm HPCh1_projectXX { true, 100.0f };
		OutputChannelPwm HPCh2_coolingPump { true, 100.0f };
		OutputChannelPwm HPCh3 { true };
		OutputChannelPwm HPCh4 { true };
		OutputChannelPwm D1_projectXX { true, 100.0f };
		OutputChannelPwm D2_PE_enable { false, 100.0f };
		OutputChannelPwm D3 { false, 100.0f };
		OutputChannelPwm D4 { false, 100.0f };
		OutputChannel SDC { true };
	};
private:

	can::RxMessageQueue<can::messages::PDU_RX_Control_NoOD> m_rxControllQueue;
	can::RxMessageQueue<can::messages::PDU_RX_Manual_Control> m_manualControllQueue;
	can::RxMessageQueue<can::messages::PDU_RX_HP_D_Dutycycle_NoOD> m_hpDutycycleQueue;
	can::RxMessageQueue<can::messages::PDU_RX_LP_Dutycycle_NoOD> m_lpDutycycleQueue;
	PduOutputState m_outputState;

	Watchdog m_stateMaschineWatchdog { OD_watchdogTimeout_get() };
	bool m_enabled;
	TickType_t m_disableTime = 0;

	AdcChannelController m_lpChannelAdc[10];
	AdcChannelController m_hpChannelAdc[4];
	MovingAverageFilter m_lpAverage[10];
	MovingAverageFilter m_hpAverage[4];
	float m_lastDataRead = 0;
	float m_lastStatusSent = 0;

	bool m_lastPduEnabled = false;
	bool m_lastPeHwEnabled = false;
	bool m_lastErrorFlag = false;

};
