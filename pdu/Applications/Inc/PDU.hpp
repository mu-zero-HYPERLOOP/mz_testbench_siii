/*
 * PDU.hpp
 *
 *  Created on:
 *      Author:
 *
 */
#pragma once

#include "estdio.hpp"
#include "main.h"
#include "canzero.hpp"
#include "Watchdog.hpp"
#include <cmath>
#include <algorithm>
#include "AdcDma.hpp"
#include "adc.h"
#include "led.hpp"
#include "tim.h"
#include "AdcChannelController.hpp"

//AdcDma<4> adc1 { &hadc1 }; //ADC class to read basic BCU signals, already defined in TaskManager.cpp
//AdcDma<14> adc2 = AdcDma<14>(&hadc2); // ADC class to read the current of the channels


AdcChannelController lpChannelAdc[10];
AdcChannelController hpChannelAdc[4];

float lpChannelCurrent[10];	// Current of the ten BTF6070 channels in A
float hpChannelCurrent[4];	// Current of the four BTT6010 channels in A
float lvCurrent = 0;		// Total current of the LV battery in A
int ledCounter = 0;			// Counter to blink the status LED
RxMessage rxRawMsg;			// Buffer to receive a CAN message
//extern MessageBufferHandle_t handlePduRxMessageBuffer;

// Flags from the State Machine
bool pduEnabled = false;
bool errorReset = false;
bool peHwEnable = false;
bool ledState = false;

bool lastPduEnabled = false;
bool lastPeHwEnabled = false;
bool lastErrorFlag = false;

float batteryTemperature = 0;

can::RxMessageQueue<can::messages::PDU_RX_Control_NoOD> g_rxControllQueue;
can::RxMessageQueue<can::messages::PDU_RX_Manual_Control> g_manualControllQueue;
can::RxMessageQueue<can::messages::PDU_RX_HP_D_Dutycycle_NoOD> g_hpDutycycleQueue;
can::RxMessageQueue<can::messages::PDU_RX_LP_Dutycycle_NoOD> g_lpDutycycleQueue;
//can::RxMessageQueue<can::messages::Sensor_TX_BMS> g_bmsQueue;

Watchdog stateMachineWatchdog(OD_watchdogTimeout_get());

// Number of millisecconds the PE pump should stay on after the PE is disabled
TickType_t disableTime = 0;
constexpr TickType_t pePumpFollowUpTimeMs = 60000;

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
		if (xTaskGetTickCount() - m_lastChange >= m_waitingPeriodAfterChange) {
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
				&& xTaskGetTickCount() - m_lastChange >= 20) {//  status != OUTPUT_STATE::STATUS_CHANGE_PENDING
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
	;
};

// All output channels of the PDU with the initial states
typedef struct PduOutputState {
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
} PduOutputState;

PduOutputState outputState;

extern MessageBufferHandle_t handlePduRxStreamBuffer;

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

/**
 * Blink the status LED on the PDU
 */
void blinkStatusLed() {
	if (ledCounter == 0) {
		ledState = !ledState;
		if (ledState) {
			LED_Green_Write(150);
		} else {
			LED_Green_Write(0);
		}
	}
	ledCounter = (ledCounter + 1) % (pduEnabled ? 20 : 100);
}

// Celldata array need to be in ascending order to work with linear interpolation!
constexpr uint16_t LIPO_CELL_DATA_NUM_POINTS = 21;
constexpr float LIPO_CELL_DATA[LIPO_CELL_DATA_NUM_POINTS][2] = {
		{ 3.27f, 0.0f }, { 3.61f, 5.0f }, { 3.69f, 10.0f }, { 3.71f, 15.0f }, {
				3.73f, 20.0f }, { 3.75f, 25.0f }, { 3.77f, 30.0f }, { 3.79f,
				35.0f }, { 3.80f, 40.0f }, { 3.82f, 45.0f }, { 3.84f, 50.0f }, {
				3.85f, 55.0f }, { 3.87f, 60.0f }, { 3.91f, 65.0f }, { 3.95f,
				70.0f }, { 3.98f, 75.0f }, { 4.02f, 80.0f }, { 4.08f, 85.0f }, {
				4.11f, 90.0f }, { 4.15f, 95.0f }, { 4.20f, 100.0f }, };

/**
 * Estimate the SOC of a LiPo battery based on a single cell voltage.
 * It linearly interpolates the given array of cell voltage and SOC.
 * @param cellVoltage
 * @return
 */
float estimateLiPoSoc(float cellVoltage) {
	uint16_t i = 0;
	for (i = 0; i < LIPO_CELL_DATA_NUM_POINTS; i++) {
		if (cellVoltage <= LIPO_CELL_DATA[i][0]) {
			break;
		}
	}
	// Value is smaller than the smallest value
	if (i == 0) {
		return LIPO_CELL_DATA[i][1];
	}

	// Value is bigger than the biggest value
	if (i == LIPO_CELL_DATA_NUM_POINTS) {
		return LIPO_CELL_DATA[i - 1][1];
	}

	// Value is between two values
	return (cellVoltage - LIPO_CELL_DATA[i - 1][0])
			/ (LIPO_CELL_DATA[i][0] - LIPO_CELL_DATA[i - 1][0])
			* (LIPO_CELL_DATA[i][1] - LIPO_CELL_DATA[i - 1][1])
			+ LIPO_CELL_DATA[i - 1][1];
}

/**
 * Function to receive all CAN messages
 */

void receiveCanMessages() {
	using namespace can;

	while (g_rxControllQueue.hasAny()) {
		printf("received rx controll frame\n");
		// Reset watchdog for state machine
		stateMachineWatchdog.reset();

		can::Message<messages::PDU_RX_Control_NoOD> controlMsg =
				g_rxControllQueue.dequeue();
		bool errorReset = controlMsg.get<signals::PDU_RX_ErrorReset>();
		bool _enable = controlMsg.get<signals::PDU_RX_Enable>(); //activates control mode
		bool peHwEnable = controlMsg.get<signals::PDU_RX_PEHWEnable>(); //allows the PDU to set the PE_enable channel (D2)

		// Reset all errors if requested
		if (errorReset) {
			resetAllErrors();
		}

		// Edge detection: Mode change from Manual Control to State Machine Control
		if (!pduEnabled && _enable) { // Enable was just set
			if (!anyErrorPresent()) { // Do not enable if there is still an error present
				pduEnabled = _enable;

				// Reset all output channels to their default values:
				outputState = { };
			}

			// Set CANzero status to operational -> changing of critical OD entries now locked
			canzero::setStatus(operational);
		}

		// Edge detection: Mode change from State Machine Control to Manual Control
		if (pduEnabled && !_enable) { // Enable was reset
			pduEnabled = _enable;

			// Reset all output channels to their default values:
			outputState = { };

			// Set CANzero status to default state
			canzero::setStatus(pre_operational);

			// Let cooling pump run until one minute after disabling
			disableTime = xTaskGetTickCount();
		}

		// PDU is pduEnabled and in State Machine control mode
		if (pduEnabled) {
			outputState.SDC.set(true);
			outputState.LPCh5_powerElectronics.set(true);
			outputState.LPCh6.set(peHwEnable);
		}
		printf("pdu enabled = %u\n", pduEnabled);
	}

	while (g_manualControllQueue.hasAny()) {
		printf("manuel controll\n");
		can::Message<messages::PDU_RX_Manual_Control> manualControlMsg =
				g_manualControllQueue.dequeue();

		// PDU is in manual control mode
		if (!pduEnabled) {
			outputState.LPCh1_opticalSensor.set(
					manualControlMsg.get<signals::PDU_LPCh1_Enable>());
			outputState.LPCh2.set(
					manualControlMsg.get<signals::PDU_LPCh2_Enable>());
			outputState.LPCh3_HVCU.set(
					manualControlMsg.get<signals::PDU_LPCh3_Enable>());
			outputState.LPCh4_frontECU.set(
					manualControlMsg.get<signals::PDU_LPCh4_Enable>());
			outputState.LPCh5_powerElectronics.set(
					manualControlMsg.get<signals::PDU_LPCh5_Enable>());
			outputState.LPCh6.set(
					manualControlMsg.get<signals::PDU_LPCh6_Enable>());
			outputState.LPCh7_rearECU.set(
					manualControlMsg.get<signals::PDU_LPCh7_Enable>());
			outputState.LPCh8_telemetry.set(
					manualControlMsg.get<signals::PDU_LPCh8_Enable>());
			outputState.LPCh9_logger.set(
					manualControlMsg.get<signals::PDU_LPCh9_Enable>());
			outputState.LPCh10.set(
					manualControlMsg.get<signals::PDU_LPCh10_Enable>());

			outputState.HPCh1_projectXX.set(
					manualControlMsg.get<signals::PDU_HPCh1_Enable>());
			outputState.HPCh2_coolingPump.set(
					manualControlMsg.get<signals::PDU_HPCh2_Enable>());
			outputState.HPCh3.set(
					manualControlMsg.get<signals::PDU_HPCh3_Enable>());
			outputState.HPCh4.set(
					manualControlMsg.get<signals::PDU_HPCh4_Enable>());

			outputState.D1_projectXX.set(
					manualControlMsg.get<signals::PDU_D1_Enable>());
			outputState.D2_PE_enable.set(
					manualControlMsg.get<signals::PDU_D2_Enable>());
			outputState.D3.set(manualControlMsg.get<signals::PDU_D3_Enable>());
			outputState.D4.set(manualControlMsg.get<signals::PDU_D4_Enable>());

			outputState.SDC.set(
					manualControlMsg.get<signals::PDU_SDC_Enable>());
		}
	}

	while (g_lpDutycycleQueue.hasAny()) {
		printf("lp duty\n");
		can::Message<messages::PDU_RX_LP_Dutycycle_NoOD> dutyMsg =
				g_lpDutycycleQueue.dequeue();
		// Update duty cycles when PDU is in manual control mode
		if (!pduEnabled) {
			outputState.LPCh1_opticalSensor.setDuty(
					dutyMsg.get<signals::PDU_LPCh1_Dutycycle>());
			outputState.LPCh2.setDuty(
					dutyMsg.get<signals::PDU_LPCh2_Dutycycle>());
			outputState.LPCh3_HVCU.setDuty(
					dutyMsg.get<signals::PDU_LPCh3_Dutycycle>());
			outputState.LPCh8_telemetry.setDuty(
					dutyMsg.get<signals::PDU_LPCh8_Dutycycle>());
			outputState.LPCh9_logger.setDuty(
					dutyMsg.get<signals::PDU_LPCh9_Dutycycle>());
			outputState.LPCh10.setDuty(
					dutyMsg.get<signals::PDU_LPCh10_Dutycycle>());
		}
	}

	while (g_hpDutycycleQueue.hasAny()) {
		printf("hp duty\n");
		can::Message<messages::PDU_RX_HP_D_Dutycycle_NoOD> dutyMsg =
				g_hpDutycycleQueue.dequeue();
		if (!pduEnabled) {
			outputState.HPCh1_projectXX.setDuty(
					dutyMsg.get<signals::PDU_HPCh1_Dutycycle>());
			outputState.HPCh2_coolingPump.setDuty(
					dutyMsg.get<signals::PDU_HPCh2_Dutycycle>());
			outputState.D1_projectXX.setDuty(
					dutyMsg.get<signals::PDU_D1_Dutycycle>());
			outputState.D2_PE_enable.setDuty(
					dutyMsg.get<signals::PDU_D2_Dutycycle>());
			outputState.D3.setDuty(dutyMsg.get<signals::PDU_D3_Dutycycle>());
			outputState.D4.setDuty(dutyMsg.get<signals::PDU_D4_Dutycycle>());
		}
	}

	/*
	while (g_bmsQueue.hasAny()) {
		can::Message<messages::Sensor_TX_BMS> batteryTempMsg =
				g_bmsQueue.dequeue();
		batteryTemperature =
				batteryTempMsg.get<signals::Sensor_TX_BatteryTemp>();
	}
	*/
}

// Read the current of all channels and send it
void readAndSendData() {
	using namespace can;
	// Start reading all channels of ADC2
	//adc2.start();
	//ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000));

	//float *adcData = adc2.getData();

	lvCurrent = 0;
	for (int i = 0; i < 10; i++) { //maybe the values for i needs to be changed because the ADC channels have changed.
		// Raw ADC value is converted into a voltage
		// The sense current is calculated through the 62Ohms current sense resistor
		// The sense current is multiplied by the current divider factor of BTF6070
		lpChannelCurrent[i] = (lpChannelAdc[i].get() * 3.3f / 4095.0f) / 62.0f * 1750
				- 0.010f;	// Subtract 10mA since this seems to be an offset

		// Limit to plausible values
		if (lpChannelCurrent[i] < 0.0f) {
			lpChannelCurrent[i] = 0.0f;
		}
		if (lpChannelCurrent[i] > 8.0f) {
			lpChannelCurrent[i] = 8.0f;
		}

		// Sum up current to total LV battery current
		if (lpChannelCurrent[i] < 8.0f) {
			// Do not sum up error current values
			lvCurrent += lpChannelCurrent[i];
		}
	}

	for (int i = 0; i < 4; i++) {
		// Raw ADC value is converted into a voltage
		// The sense current is calculated through the 62Ohms current sense resistor
		// The sense current is multiplied by the current divider factor of BTT6010
		hpChannelCurrent[i] = (hpChannelAdc[i].get() * 3.3f / 4095.0f) / 62.0f * 4000
				- 0.300f;	// Subtract 300mA since this seems to be an offset

		// Limit to plausible values
		if (hpChannelCurrent[i] < 0.0f) {
			hpChannelCurrent[i] = 0.0f;
		}
		if (hpChannelCurrent[i] > 16.0f) {
			hpChannelCurrent[i] = 16.0f;
		}

		// Sum up current to total LV battery current
		if (hpChannelCurrent[i] < 16.0f) {
			// Do not sum up error current values
			lvCurrent += hpChannelCurrent[i];
		}
	}

	outputState.LPCh1_opticalSensor.setCurrent(lpChannelCurrent[0]);
	outputState.LPCh2.setCurrent(lpChannelCurrent[1]);
	outputState.LPCh3_HVCU.setCurrent(lpChannelCurrent[2]);
	outputState.LPCh4_frontECU.setCurrent(lpChannelCurrent[3]);
	outputState.LPCh5_powerElectronics.setCurrent(lpChannelCurrent[4]);
	outputState.LPCh6.setCurrent(lpChannelCurrent[5]);
	outputState.LPCh7_rearECU.setCurrent(lpChannelCurrent[6]);
	outputState.LPCh8_telemetry.setCurrent(lpChannelCurrent[7]);
	outputState.LPCh9_logger.setCurrent(lpChannelCurrent[8]);
	outputState.LPCh10.setCurrent(lpChannelCurrent[9]);

	outputState.HPCh1_projectXX.setCurrent(hpChannelCurrent[0]);
	outputState.HPCh2_coolingPump.setCurrent(hpChannelCurrent[1]);
	outputState.HPCh3.setCurrent(hpChannelCurrent[2]);
	outputState.HPCh4.setCurrent(hpChannelCurrent[3]);

	// Range limit, otherwise sending the CAN message may fail
	if (lvCurrent > 40) {
		lvCurrent = 40;
	}
	if (lvCurrent < 0) {
		lvCurrent = 0;
	}

	// Estimate SOC
	float cellVoltage = OD_InputVoltage_get() / 6.0f;
	int lvSoc = std::roundf(estimateLiPoSoc(cellVoltage));
	lvSoc = std::min(std::max(lvSoc, 0), 100);

	// Current of LPchannels 1 to 5
	Message<messages::PDU_TX_LP_Current1> msgCurrent1;
	msgCurrent1.set<signals::PDU_LPCh1_Current>(lpChannelCurrent[0]);
	msgCurrent1.set<signals::PDU_LPCh2_Current>(lpChannelCurrent[1]);
	msgCurrent1.set<signals::PDU_LPCh3_Current>(lpChannelCurrent[2]);
	msgCurrent1.set<signals::PDU_LPCh4_Current>(lpChannelCurrent[3]);
	msgCurrent1.set<signals::PDU_LPCh5_Current>(lpChannelCurrent[4]);
	msgCurrent1.send();

	osDelay(pdMS_TO_TICKS(1));// TODO when cz_interface is fixed, this can be removed

	// Current of LPchannels 6 to 10
	Message<messages::PDU_TX_LP_Current2> msgCurrent2;
	msgCurrent2.set<signals::PDU_LPCh6_Current>(lpChannelCurrent[5]);
	msgCurrent2.set<signals::PDU_LPCh7_Current>(lpChannelCurrent[6]);
	msgCurrent2.set<signals::PDU_LPCh8_Current>(lpChannelCurrent[7]);
	msgCurrent2.set<signals::PDU_LPCh9_Current>(lpChannelCurrent[8]);
	msgCurrent2.set<signals::PDU_LPCh10_Current>(lpChannelCurrent[9]);
	msgCurrent2.send();

	// Current of HPchannels 1 to 4
	Message<messages::PDU_TX_HP_Current> msgCurrent3;
	msgCurrent3.set<signals::PDU_HPCh1_Current>(hpChannelCurrent[0]);
	msgCurrent3.set<signals::PDU_HPCh2_Current>(hpChannelCurrent[1]);
	msgCurrent3.set<signals::PDU_HPCh3_Current>(hpChannelCurrent[2]);
	msgCurrent3.set<signals::PDU_HPCh4_Current>(hpChannelCurrent[3]);
	msgCurrent3.send();

	// LV battery data
	Message<messages::PDU_TX_LV_BMS> msgBattery;
	msgBattery.set<signals::PDU_LV_Voltage>(OD_InputVoltage_get());
	msgBattery.set<signals::PDU_LV_Current>(lvCurrent);
	msgBattery.set<signals::PDU_LV_SOC>(lvSoc);
	msgBattery.send();

	if (OD_CoolingPumpEnabled
			!= can::signals::PDU_OD_CoolingPumpEnabled::ENABLE) {
		outputState.HPCh2_coolingPump.set(false);
	}

	// Short Circuit Debug Message
	Message<messages::PDU_TX_LP_Short_Circuit_Debug> msgDebug1;
	msgDebug1.set<signals::PDU_LPCh1_ShortCnt>(
			outputState.LPCh1_opticalSensor.numShorts());
	msgDebug1.set<signals::PDU_LPCh1_State>(
			outputState.LPCh1_opticalSensor.getStatus());
	msgDebug1.set<signals::PDU_LPCh2_ShortCnt>(outputState.LPCh2.numShorts());
	msgDebug1.set<signals::PDU_LPCh2_State>(outputState.LPCh2.getStatus());
	msgDebug1.set<signals::PDU_LPCh3_ShortCnt>(
			outputState.LPCh3_HVCU.numShorts());
	msgDebug1.set<signals::PDU_LPCh3_State>(outputState.LPCh3_HVCU.getStatus());
	msgDebug1.set<signals::PDU_LPCh4_ShortCnt>(
			outputState.LPCh4_frontECU.numShorts());
	msgDebug1.set<signals::PDU_LPCh4_State>(
			outputState.LPCh4_frontECU.getStatus());
	msgDebug1.set<signals::PDU_LPCh5_ShortCnt>(
			outputState.LPCh5_powerElectronics.numShorts());
	msgDebug1.set<signals::PDU_LPCh5_State>(
			outputState.LPCh5_powerElectronics.getStatus());
	msgDebug1.set<signals::PDU_LPCh6_ShortCnt>(outputState.LPCh6.numShorts());
	msgDebug1.set<signals::PDU_LPCh6_State>(outputState.LPCh6.getStatus());
	msgDebug1.set<signals::PDU_LPCh7_ShortCnt>(
			outputState.LPCh7_rearECU.numShorts());
	msgDebug1.set<signals::PDU_LPCh7_State>(
			outputState.LPCh7_rearECU.getStatus());
	msgDebug1.set<signals::PDU_LPCh8_ShortCnt>(
			outputState.LPCh8_telemetry.numShorts());
	msgDebug1.set<signals::PDU_LPCh8_State>(
			outputState.LPCh8_telemetry.getStatus());
	msgDebug1.set<signals::PDU_LPCh9_ShortCnt>(
			outputState.LPCh9_logger.numShorts());
	msgDebug1.set<signals::PDU_LPCh9_State>(
			outputState.LPCh9_logger.getStatus());
	msgDebug1.set<signals::PDU_LPCh10_ShortCnt>(outputState.LPCh10.numShorts());
	msgDebug1.set<signals::PDU_LPCh10_State>(outputState.LPCh10.getStatus());
	msgDebug1.send();

	Message<messages::PDU_TX_HP_Short_Circuit_Debug> msgDebug2;
	msgDebug2.set<signals::PDU_HPCh1_ShortCnt>(
			outputState.HPCh1_projectXX.numShorts());
	msgDebug2.set<signals::PDU_HPCh1_State>(
			outputState.HPCh1_projectXX.getStatus());
	msgDebug2.set<signals::PDU_HPCh2_ShortCnt>(
			outputState.HPCh2_coolingPump.numShorts());
	msgDebug2.set<signals::PDU_HPCh2_State>(
			outputState.HPCh2_coolingPump.getStatus());
	msgDebug2.set<signals::PDU_HPCh3_ShortCnt>(outputState.HPCh3.numShorts());
	msgDebug2.set<signals::PDU_HPCh3_State>(outputState.HPCh3.getStatus());
	msgDebug2.set<signals::PDU_HPCh4_ShortCnt>(outputState.HPCh4.numShorts());
	msgDebug2.set<signals::PDU_HPCh4_State>(outputState.HPCh4.getStatus());
	msgDebug2.send();
}

/**
 * Safety checks for the battery: Low voltage warning, critical voltage error, over current error.
 */
void batterySafetyChecks() {
	// Counters to track how long an error is present
	static uint16_t errorUndervoltageCounter = 0;
	static uint16_t errorOvercurrentCounter = 0;
	static uint16_t errorOvertemperatureCounter = 0;

	// Warning when battery gets low
	if (OD_InputVoltage_get() < OD_batterVoltageLow_get()) {
		WARN_batterVoltageLow_set();
	} else {
		WARN_batterVoltageLow_clear();
	}

	//Warning when battery temperature rises too high
	if (batteryTemperature > OD_overTempWarn_get()) {
		WARN_batterTempHigh_set();
	} else {
		WARN_batterTempHigh_clear();
	}

	// Error and shutdown when LV battery voltage gets critical low
	if (OD_InputVoltage_get() < OD_batterVoltageCritical_get()
			|| errorUndervoltageCounter >= 200) {

		errorUndervoltageCounter++;

		// If error is present for 100 or more cycles (500ms), set the error
		if (errorUndervoltageCounter == 100) {
			ERR_batterVoltageCritical_set();

			// If undervoltage is longer than 1s, shut everything down
		} else if (errorUndervoltageCounter >= 200) {
			outputState.LPCh1_opticalSensor.set(false);
			outputState.LPCh2.set(false);
			outputState.LPCh3_HVCU.set(false);
			outputState.LPCh4_frontECU.set(false);
			outputState.LPCh5_powerElectronics.set(false);
			outputState.LPCh6.set(false);
			outputState.LPCh7_rearECU.set(false);
			outputState.LPCh8_telemetry.set(false);
			outputState.LPCh9_logger.set(false);
			outputState.LPCh10.set(false);
			outputState.HPCh1_projectXX.set(false);
			outputState.HPCh2_coolingPump.set(false);
			outputState.HPCh3.set(false);
			outputState.HPCh4.set(false);
			outputState.D1_projectXX.set(false);
			outputState.D2_PE_enable.set(false);
			outputState.D3.set(false);
			outputState.D4.set(false);
			outputState.SDC.set(false);
		}
	} else {
		errorUndervoltageCounter = 0;
	}

	if (batteryTemperature > OD_overTempCritical_get()
			|| errorOvertemperatureCounter >= 400) {
		errorOvertemperatureCounter++;

		// If error is present for 200 or more cycles (1000ms), set the error
		if (errorOvertemperatureCounter == 200) {
			ERR_batterTempCritical_set();

			// If overtemperature is longer than 2s, shut everything down
		} else if (errorOvertemperatureCounter >= 400) {
			outputState.LPCh1_opticalSensor.set(false);
			outputState.LPCh2.set(false);
			outputState.LPCh3_HVCU.set(false);
			outputState.LPCh4_frontECU.set(false);
			outputState.LPCh5_powerElectronics.set(false);
			outputState.LPCh6.set(false);
			outputState.LPCh7_rearECU.set(false);
			outputState.LPCh8_telemetry.set(false);
			outputState.LPCh9_logger.set(false);
			outputState.LPCh10.set(false);
			outputState.HPCh1_projectXX.set(false);
			outputState.HPCh2_coolingPump.set(false);
			outputState.HPCh3.set(false);
			outputState.HPCh4.set(false);
			outputState.D1_projectXX.set(false);
			outputState.D2_PE_enable.set(false);
			outputState.D3.set(false);
			outputState.D4.set(false);
			outputState.SDC.set(false);
		}
	} else {
		errorOvertemperatureCounter = 0;
	}

	// Error and shutdown when LV battery current exceeds critical value
	if (lvCurrent > OD_batteryOvercurrent_get()
			|| errorOvercurrentCounter > 10) {

		errorOvercurrentCounter++;

		// If error is present for 10 or more cycles, shut down pod and set the error
		if (errorOvercurrentCounter > 10) {
			ERR_batteryOvercurrent_set();
			outputState.LPCh1_opticalSensor.set(false);
			outputState.LPCh2.set(false);
			outputState.LPCh3_HVCU.set(false);
			outputState.LPCh4_frontECU.set(false);
			outputState.LPCh5_powerElectronics.set(false);
			outputState.LPCh6.set(false);
			outputState.LPCh7_rearECU.set(false);
			outputState.LPCh8_telemetry.set(false);
			outputState.LPCh9_logger.set(false);
			outputState.LPCh10.set(false);
			outputState.HPCh1_projectXX.set(false);
			outputState.HPCh2_coolingPump.set(false);
			outputState.HPCh3.set(false);
			outputState.HPCh4.set(false);
			outputState.D1_projectXX.set(false);
			outputState.D2_PE_enable.set(false);
			outputState.D3.set(false);
			outputState.D4.set(false);
			outputState.SDC.set(false);
		}
	} else {
		errorOvercurrentCounter = 0;
	}
}

/**
 * Set the hardware channels
 */
void updateChannels() {
	outputState.LPCh1_opticalSensor.update();
	outputState.LPCh2.update();
	outputState.LPCh3_HVCU.update();
	outputState.LPCh4_frontECU.update();
	outputState.LPCh5_powerElectronics.update();
	outputState.LPCh6.update();
	outputState.LPCh7_rearECU.update();
	outputState.LPCh8_telemetry.update();
	outputState.LPCh9_logger.update();
	outputState.LPCh10.update();
	outputState.HPCh1_projectXX.update();
	outputState.HPCh2_coolingPump.update();
	outputState.HPCh3.update();
	outputState.HPCh4.update();
	outputState.D1_projectXX.update();
	outputState.D2_PE_enable.update();
	outputState.D3.update();
	outputState.D4.update();

	// Standard output channels with PWM support

	//Somehow all timers except of TIM2 can't save dutycycles of 100% so that the +1 was deleted.

	// LPCh1 is TIM12_CH2
	if (outputState.LPCh1_opticalSensor.getSwitch()) {
		htim12.Instance->CCR2 = outputState.LPCh1_opticalSensor.getDuty()
				* (htim12.Instance->ARR) / 100.0f;
	} else {
		htim12.Instance->CCR2 = 0;
	}

	// LPCh2 is TIM2_CH3
	if (outputState.LPCh2.getSwitch()) {
		htim2.Instance->CCR3 = outputState.LPCh2.getDuty()
				* (htim2.Instance->ARR + 1) / 100.0f;
	} else {
		htim2.Instance->CCR3 = 0;
	}

	// LPCh3 is TIM2_CH1
	if (outputState.LPCh3_HVCU.getSwitch()) {
		htim2.Instance->CCR1 = outputState.LPCh3_HVCU.getDuty()
				* (htim2.Instance->ARR + 1) / 100.0f;
	} else {
		htim2.Instance->CCR1 = 0;
	}

	// LPCh8 is TIM8_CH1
	if (outputState.LPCh8_telemetry.getSwitch()) {
		htim8.Instance->CCR1 = outputState.LPCh3_HVCU.getDuty()
				* (htim8.Instance->ARR) / 100.0f;
	} else {
		htim8.Instance->CCR1 = 0;
	}

	// LPCh9 is TIM4_CH2
	if (outputState.LPCh9_logger.getSwitch()) {
		htim4.Instance->CCR2 = outputState.LPCh9_logger.getDuty()
				* (htim4.Instance->ARR) / 100.0f;
	} else {
		htim4.Instance->CCR2 = 0;
	}

	// LPCh10 is TIM11_CH1
	if (outputState.LPCh10.getSwitch()) {
		htim11.Instance->CCR1 = outputState.LPCh10.getDuty()
				* (htim11.Instance->ARR) / 100.0f;
	} else {
		htim11.Instance->CCR1 = 0;
	}

	// HPCh1 is TIM8_CH4
	if (outputState.HPCh1_projectXX.getSwitch()) {
		htim8.Instance->CCR4 = outputState.HPCh1_projectXX.getDuty()
				* (htim8.Instance->ARR) / 100.0f;
	} else {
		htim8.Instance->CCR4 = 0;
	}

	// HPCh2 is TIM8_CH2
	if (outputState.HPCh2_coolingPump.getSwitch()) {
		htim8.Instance->CCR2 = outputState.HPCh2_coolingPump.getDuty()
				* (htim8.Instance->ARR) / 100.0f;
	} else {
		htim8.Instance->CCR2 = 0;
	}

	// D1 is TIM3_CH1 (is controlled by ProjectXX.hpp)
	if (outputState.D1_projectXX.getSwitch()) {
		//htim3.Instance->CCR1 = outputState.D1_projectXX.getDuty() * (htim3.Instance->ARR) / 100.0f;
	} else {
		//htim3.Instance->CCR1= 0;
	}

	// D2 is TIM10_CH1
	if (outputState.D2_PE_enable.getSwitch()) {
		htim10.Instance->CCR1 = outputState.D2_PE_enable.getDuty()
				* (htim10.Instance->ARR) / 100.0f;
	} else {
		htim10.Instance->CCR1 = 0;
	}

	// D3 is TIM2_CH4
	if (outputState.D3.getSwitch()) {
		htim2.Instance->CCR4 = outputState.D3.getDuty() * (htim2.Instance->ARR)
				/ 100.0f;
	} else {
		htim2.Instance->CCR4 = 0;
	}

	// D4 is TIM3_CH3
	if (outputState.D4.getSwitch()) {
		htim3.Instance->CCR3 = outputState.D4.getDuty() * (htim3.Instance->ARR)
				/ 100.0f;
	} else {
		htim3.Instance->CCR3 = 0;
	}

	// Standard On/Off output
	HAL_GPIO_WritePin(LP4_control_GPIO_Port, LP4_control_Pin,
			outputState.LPCh4_frontECU.getSwitch() ?
					GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LP5_control_GPIO_Port, LP5_control_Pin,
			outputState.LPCh5_powerElectronics.getSwitch() ?
					GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LP6_control_GPIO_Port, LP6_control_Pin,
			outputState.LPCh6.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LP7_control_GPIO_Port, LP7_control_Pin,
			outputState.LPCh7_rearECU.getSwitch() ?
					GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HP3_control_GPIO_Port, HP3_control_Pin,
			outputState.HPCh3.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HP4_control_GPIO_Port, HP4_control_Pin,
			outputState.HPCh4.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// SDC switch
	HAL_GPIO_WritePin(SDC_control_GPIO_Port, SDC_control_Pin,
			outputState.SDC.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * Send out CAN messages
 */
void sendData() {
	using namespace can;
	// PDU status
	Message<messages::PDU_TX_Status> msgStatus;
	msgStatus.set<signals::PDU_TX_Enabled>(pduEnabled);
	msgStatus.set<signals::PDU_TX_PEHWEnabled>(outputState.LPCh6.get());
	msgStatus.set<signals::PDU_TX_ErrorFlag>(anyErrorPresent());
	msgStatus.send();
}

uint32_t lastDataRead = 0;
uint32_t lastStatusSent = 0;

uint32_t tempLast = 0;

// Main Task of the PDU
static void pduAppFunction(void *pvArguments) {

	//g_bmsQueue.enable();
	g_rxControllQueue.enable();
	g_manualControllQueue.enable();
	g_lpDutycycleQueue.enable();
	g_hpDutycycleQueue.enable();


	LED_RGB_Write(0, 0, 0);


	// Wait 100ms to be sure that battery voltage was already read at least once by the TaskManager.cpp
	osDelay(pdMS_TO_TICKS(100));


	//TODO implement adc moving average filters.
	for(size_t i = 0;i<10;i++){
		lpChannelAdc[i] = AdcChannelController(ADC_MODULE2, i);
	}
	for(size_t i=0;i<4;i++){
		hpChannelAdc[i] = AdcChannelController(ADC_MODULE2, 10 + i);
	}
	//adc2.init(50);	// Read each channel 50 times and average
	//osDelay(pdMS_TO_TICKS(25));	// Wait 25ms to ensure that ADC is initialized


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


	// Initial read of data
	readAndSendData();


	// Reset watchdog
	stateMachineWatchdog.reset();


	// Let the state machine know that we finished the ECU setup and are ready for operation
	canzero::setStatus(pre_operational);

	while (true) {
		// Blink the status LEDs
		blinkStatusLed();

		// Receive control messages
		receiveCanMessages();


		// Read channel currents, send the CAN messages and update the cooling PWM
		if (xTaskGetTickCount() - lastDataRead
				>= OD_currentReadInterval_get()) {
			lastDataRead = xTaskGetTickCount();

			// Read the current of all channels and send it
			readAndSendData();
		}

		// Error monitoring
		if (pduEnabled == false) {
			batterySafetyChecks();
		}
		if (pduEnabled == true) {
			batterySafetyChecks();
			if (stateMachineWatchdog.isTimedOut()) {
				ERR_watchdogStateMachine_set();
			}
		}

		// If any error is present, shutdown PeHwEnable and SDC and enable red led
		if (anyErrorPresent()) {
			outputState.LPCh6.set(false);
			outputState.SDC.set(false);
			LED_Red_Write(255);
		} else {
			LED_Red_Write(0);
		}

		if (outputState.SDC.get()) {
			LED_Orange_Write(255);
		} else {
			LED_Orange_Write(0);
		}

		// Update channels: on/off and duty cycle
		updateChannels();

		// Send out status message with configured interval or when there is a change in PDU status
		if (xTaskGetTickCount() - lastStatusSent >= OD_statusSendInterval_get()
				|| lastPduEnabled != pduEnabled
				|| lastPeHwEnabled != outputState.LPCh6.get()
				|| lastErrorFlag != anyErrorPresent()) {

			lastPduEnabled = pduEnabled;
			lastPeHwEnabled = outputState.LPCh6.get();
			lastErrorFlag = anyErrorPresent();

			lastStatusSent = xTaskGetTickCount();

			// Send out data to CAN bus
			sendData();
		}

		// Send out status message with configured interval or when there is a change in PDU status
		if (xTaskGetTickCount() - tempLast >= 1000){

			tempLast = xTaskGetTickCount();

			// Send out data to CAN bus
			printf("HpCh2 switch = %u, on = %u\n", outputState.HPCh2_coolingPump.getSwitch(), outputState.HPCh2_coolingPump.get());
		}

		// Delay until next check
		osDelay(pdMS_TO_TICKS(5));
	}
}