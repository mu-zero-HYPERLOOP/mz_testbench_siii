/*
 * PDU.hpp
 *
 *  Created on:
 *      Author:
 *
 */

#ifndef INCLUDE_PDU_HPP_
#define INCLUDE_PDU_HPP_

#include "main.h"
#include "cz_interface.hpp"
#include "dbc_parser.hpp"
#include "Watchdog.hpp"
#include <cmath>
#include <algorithm>
#include "estdio.hpp"
#include "GlobalState.hpp"

using namespace can;

//AdcDma<4> adc1 { &hadc1 }; //ADC class to read basic BCU signals, already defined in TaskManager.cpp
AdcDma<14> adc2 { &hadc2 };	// ADC class to read the current of the channels

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

	using OUTPUT_STATE = signals::PDU_LPCh1_State;

public:
	OutputChannelPwm(bool _on = false, float _duty = 100.0f) : m_on {_on}, m_duty{_duty}, m_switchOn{_on} {
		m_lastChange = xTaskGetTickCount();
	}

	void set(bool _on, float _duty = -100.0f) {
		if(_on != m_on || (_duty >= 0.0f && _duty != m_duty)) {
			m_lastChange = xTaskGetTickCount();
			m_on = _on;
			if(m_duty >= 0.0f) {
				m_duty = _duty;
			}
			m_switchOn = m_on;

			if(!m_on) {
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
		if(xTaskGetTickCount() - m_lastChange >= m_waitingPeriodAfterChange) {
			if(m_current >= m_errorCurrent) {
				if(m_on) {
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

		if(status == OUTPUT_STATE::OUTPUT_SHORT_CIRCUIT) {
			if(m_shortCircuitsDetected <= m_shortCircuitResetAttempts) {
				m_shortCircuitsDetected++;

				m_switchOn = false;
				m_lastChange = xTaskGetTickCount();
				m_shortCircuitResetOngoing = true;
			}
		}

		status = getStatus();

		if(m_shortCircuitResetOngoing && xTaskGetTickCount() - m_lastChange >= 20) {	//  status != OUTPUT_STATE::STATUS_CHANGE_PENDING
			m_switchOn = true;
			m_lastChange = xTaskGetTickCount();
			m_shortCircuitResetOngoing = false;
		}
	}

};

class OutputChannel : public OutputChannelPwm {
public:
	OutputChannel(bool _on = false) : OutputChannelPwm{_on, 100.0f} {};
};

// All output channels of the PDU with the initial states
typedef struct PduOutputState {
	OutputChannelPwm 	LPCh1					{false, 100.0f}; // true
	OutputChannelPwm 	LPCh2					{false}; //true (SDC).
	OutputChannelPwm	LPCh3					{false, 100.0f}; // true
	OutputChannel 		LPCh4					{false};
	OutputChannel 		LPCh5					{true};  //ebox tower
	OutputChannel 		LPCh6   				{false};
	OutputChannel 		LPCh7					{false};
	OutputChannelPwm 	LPCh8					{false, 100.0f};
	OutputChannelPwm	LPCh9					{false};
	OutputChannelPwm 	LPCh10					{false, 100.0f};
	OutputChannelPwm	HPCh1					{false, 100.0f}; //led strip power
	OutputChannelPwm	HPCh2					{false, 100.0f}; //cooling pump.
	OutputChannelPwm	HPCh3					{false}; //maybe solenoid source (better for led strip because no remote)
	OutputChannelPwm	HPCh4					{false}; //reserved
	OutputChannelPwm	D1						{false, 100.0f}; //led digital
	OutputChannelPwm	D2						{false, 100.0f}; //reserved
	OutputChannelPwm	D3						{false, 100.0f}; //reserved
	OutputChannelPwm	D4						{false, 100.0f}; //reserved
	OutputChannel		SDC 					{true}; // not used!
} PduOutputState;

PduOutputState outputState;

extern MessageBufferHandle_t handlePduRxStreamBuffer;

/**
 * Checks if any error is present
 * @return true if any error is present
 */
bool anyErrorPresent() {
	return ERR_batterVoltageCritical_get() || ERR_batteryOvercurrent_get() || ERR_watchdogStateMachine_get();
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
	if(ledCounter == 0) {
		ledState = !ledState;
		if(ledState) {
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
		{3.27f,   0.0f},
		{3.61f,   5.0f},
		{3.69f,  10.0f},
		{3.71f,  15.0f},
		{3.73f,  20.0f},
		{3.75f,  25.0f},
		{3.77f,  30.0f},
		{3.79f,  35.0f},
		{3.80f,  40.0f},
		{3.82f,  45.0f},
		{3.84f,  50.0f},
		{3.85f,  55.0f},
		{3.87f,  60.0f},
		{3.91f,  65.0f},
		{3.95f,  70.0f},
		{3.98f,  75.0f},
		{4.02f,  80.0f},
		{4.08f,  85.0f},
		{4.11f,  90.0f},
		{4.15f,  95.0f},
		{4.20f, 100.0f},
};

/**
 * Estimate the SOC of a LiPo battery based on a single cell voltage.
 * It linearly interpolates the given array of cell voltage and SOC.
 * @param cellVoltage
 * @return
 */
float estimateLiPoSoc(float cellVoltage) {
	uint16_t i = 0;
	for(i = 0; i < LIPO_CELL_DATA_NUM_POINTS; i++) {
		if(cellVoltage <= LIPO_CELL_DATA[i][0]) {
			break;
		}
	}
	// Value is smaller than the smallest value
	if(i == 0) {
		return LIPO_CELL_DATA[i][1];
	}

	// Value is bigger than the biggest value
	if(i == LIPO_CELL_DATA_NUM_POINTS) {
		return LIPO_CELL_DATA[i-1][1];
	}

	// Value is between two values
	return (cellVoltage - LIPO_CELL_DATA[i-1][0]) / (LIPO_CELL_DATA[i][0] - LIPO_CELL_DATA[i-1][0]) * (LIPO_CELL_DATA[i][1] - LIPO_CELL_DATA[i-1][1]) + LIPO_CELL_DATA[i-1][1];
}


/**
 * Function to receive all CAN messages
 */

void receiveCanMessages() {
	state::receiveCAN();
	using namespace can;
	while(xMessageBufferReceive(handlePduRxMessageBuffer, &rxRawMsg, sizeof(rxRawMsg), 0) != 0) {
		// Control message from state machine
		if(checkRxMessage<messages::PDU_RX_Control>(rxRawMsg)) {
			// Reset watchdog for state machine
			stateMachineWatchdog.reset();

			can::Message<messages::PDU_RX_Control> controlMsg{rxRawMsg};
			bool errorReset = controlMsg.get<signals::PDU_RX_ErrorReset>();
			bool _enable	= controlMsg.get<signals::PDU_RX_Enable>();  //activates control mode
			//bool peHwEnable = controlMsg.get<signals::PDU_RX_PEHWEnable>(); //allows the PDU to set the PE_enable channel (D2)


			// Reset all errors if requested
			if(errorReset) {
				resetAllErrors();
			}

			// Edge detection: Mode change from Manual Control to State Machine Control
			if(!pduEnabled && _enable) { // Enable was just set
				if(!anyErrorPresent()) {	// Do not enable if there is still an error present
					pduEnabled = _enable;

					// Reset all output channels to their default values:
					outputState = {};
				}

				// Set CANzero status to operational -> changing of critical OD entries now locked
				cz_interface::getInstance()->setStatus(operational);
			}

			// Edge detection: Mode change from State Machine Control to Manual Control
			if(pduEnabled && !_enable) { // Enable was reset
				pduEnabled = _enable;

				// Reset all output channels to their default values:
				outputState = {};

				// Set CANzero status to default state
				cz_interface::getInstance()->setStatus(pre_operational);

				// Let cooling pump run until one minute after disabling
				disableTime = xTaskGetTickCount();
			}

			// PDU is pduEnabled and in State Machine control mode
			if(pduEnabled) {
				//outputState.SDC.set(true);
			}

		} else if(checkRxMessage<messages::PDU_RX_Manual_Control>(rxRawMsg)) {
			can::Message<messages::PDU_RX_Manual_Control> manualControlMsg{rxRawMsg};

			// PDU is in manual control mode
			if(!pduEnabled) {
				outputState.LPCh1.set(					manualControlMsg.get<signals::PDU_LPCh1_Enable>());
				outputState.LPCh2.set(					manualControlMsg.get<signals::PDU_LPCh2_Enable>());
				outputState.LPCh3.set(				manualControlMsg.get<signals::PDU_LPCh3_Enable>());
				outputState.LPCh4.set(			manualControlMsg.get<signals::PDU_LPCh4_Enable>());
				outputState.LPCh5.set(	manualControlMsg.get<signals::PDU_LPCh5_Enable>());
				outputState.LPCh6.set(					manualControlMsg.get<signals::PDU_LPCh6_Enable>());
				outputState.LPCh7.set(			manualControlMsg.get<signals::PDU_LPCh7_Enable>());
				outputState.LPCh8.set(  		manualControlMsg.get<signals::PDU_LPCh8_Enable>());
				outputState.LPCh9.set( 	manualControlMsg.get<signals::PDU_LPCh9_Enable>());
				outputState.LPCh10.set( 				manualControlMsg.get<signals::PDU_LPCh10_Enable>());

				outputState.HPCh1.set(	    manualControlMsg.get<signals::PDU_HPCh1_Enable>());
				outputState.HPCh2.set(		manualControlMsg.get<signals::PDU_HPCh2_Enable>());
				outputState.HPCh3.set(			 		manualControlMsg.get<signals::PDU_HPCh3_Enable>());
				outputState.HPCh4.set(					manualControlMsg.get<signals::PDU_HPCh4_Enable>());

				/*
				outputState.D1.set(			manualControlMsg.get<signals::PDU_D1_Enable>());
				outputState.D2.set(			manualControlMsg.get<signals::PDU_D2_Enable>());
				outputState.D3.set(						manualControlMsg.get<signals::PDU_D3_Enable>());
				outputState.D4.set(						manualControlMsg.get<signals::PDU_D4_Enable>());

				outputState.SDC.set(					manualControlMsg.get<signals::PDU_SDC_Enable>());
				*/
			}
		}
		else if(checkRxMessage<messages::PDU_RX_LP_Dutycycle>(rxRawMsg)) {	// Duty cycle message for manual control
			can::Message<messages::PDU_RX_LP_Dutycycle> dutyMsg{rxRawMsg};

			float lpch1_duty = dutyMsg.get<signals::PDU_LPCh1_Dutycycle>();
			outputState.LPCh1.set(lpch1_duty != 0, lpch1_duty);
			float lpch2_duty = dutyMsg.get<signals::PDU_LPCh2_Dutycycle>();
			outputState.LPCh2.set(lpch2_duty != 0,lpch2_duty);
			float lpch3_duty = dutyMsg.get<signals::PDU_LPCh3_Dutycycle>();
			outputState.LPCh3.set(lpch3_duty != 0, lpch3_duty);
			//float lpch8_duty = dutyMsg.get<signals::PDU_LPCh8_Dutycycle>();
			//outputState.LPCh8.set(lpch8_duty != 0, lpch8_duty);
			//float lpch9_duty = dutyMsg.get<signals::PDU_LPCh9_Dutycycle>();
			//outputState.LPCh9.set(lpch9_duty != 0, lpch9_duty);
			float lpch10_duty = dutyMsg.get<signals::PDU_LPCh10_Dutycycle>();
			outputState.LPCh10.set(lpch10_duty != 0, lpch10_duty);

		} else if(checkRxMessage<messages::PDU_RX_HP_Dutycycle>(rxRawMsg)) {	// Duty cycle message for manual control
			can::Message<messages::PDU_RX_HP_Dutycycle> dutyMsg{rxRawMsg};

			//float hpch1_duty = dutyMsg.get<signals::PDU_HPCh1_Dutycycle>();
			outputState.HPCh1.set(false);
			float hpch2_duty = dutyMsg.get<signals::PDU_HPCh2_Dutycycle>();
			outputState.HPCh2.set(hpch2_duty != 0, hpch2_duty);
			float hpch3_duty = dutyMsg.get<signals::PDU_HPCh3_Dutycycle>();
			outputState.HPCh3.set(hpch3_duty != 0, hpch3_duty);
			float hpch4_duty = dutyMsg.get<signals::PDU_HPCh4_Dutycycle>();
			outputState.HPCh4.set(hpch4_duty != 0, hpch4_duty);
		} else if(checkRxMessage<messages::PDU_RX_LP_Enable>(rxRawMsg)){
			can::Message<messages::PDU_RX_LP_Enable> lpEnableMsg {rxRawMsg};
			outputState.LPCh4.set(lpEnableMsg.get<signals::PDU_RX_LPCh4_Enable>());
			outputState.LPCh5.set(lpEnableMsg.get<signals::PDU_RX_LPCh5_Enable>());
			outputState.LPCh6.set(lpEnableMsg.get<signals::PDU_RX_LPCh6_Enable>());
			outputState.LPCh7.set(lpEnableMsg.get<signals::PDU_RX_LPCh7_Enable>());

		}
	}
}

// Read the current of all channels and send it
void readAndSendData() {
	// Start reading all channels of ADC2
	adc2.start();
	ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10000));
	float *adcData = adc2.getData();

	lvCurrent = 0;
	for (int i = 0; i < 10; i++) {		//maybe the values for i needs to be changed because the ADC channels have changed.
		// Raw ADC value is converted into a voltage
		// The sense current is calculated through the 62Ohms current sense resistor
		// The sense current is multiplied by the current divider factor of BTF6070
		lpChannelCurrent[i] = (adcData[i] * 3.3f / 4095.0f) / 62.0f * 1750 - 0.010f;	// Subtract 10mA since this seems to be an offset

		/*
		// Limit to plausible values
		if(lpChannelCurrent[i] < 0.0f) {
			lpChannelCurrent[i] = 0.0f;
		}
		if (lpChannelCurrent[i] > 8.0f) {
			lpChannelCurrent[i] = 8.0f;
		}
		*/

		// Sum up current to total LV battery current
		if(lpChannelCurrent[i] < 8.0f) {
			// Do not sum up error current values
			lvCurrent += lpChannelCurrent[i];
		}
	}

	for (int i = 0; i < 4; i++) {
		// Raw ADC value is converted into a voltage
		// The sense current is calculated through the 62Ohms current sense resistor
		// The sense current is multiplied by the current divider factor of BTT6010
		hpChannelCurrent[i] = (adcData[i + 10] * 3.3f / 4095.0f) / 62.0f * 4000 - 0.300f;	// Subtract 300mA since this seems to be an offset

		// Limit to plausible values
		if(hpChannelCurrent[i] < 0.0f) {
			hpChannelCurrent[i] = 0.0f;
		}
		if (hpChannelCurrent[i] > 16.0f) {
			hpChannelCurrent[i] = 16.0f;
		}

		// Sum up current to total LV battery current
		if(hpChannelCurrent[i] < 16.0f) {
			// Do not sum up error current values
			lvCurrent += hpChannelCurrent[i];
		}
	}

	outputState.LPCh1.setCurrent(lpChannelCurrent[0]);
	outputState.LPCh2.setCurrent(lpChannelCurrent[1]);
	outputState.LPCh3.setCurrent(lpChannelCurrent[2]);
	outputState.LPCh4.setCurrent(lpChannelCurrent[3]);
	outputState.LPCh5.setCurrent(lpChannelCurrent[4]);
	outputState.LPCh6.setCurrent(lpChannelCurrent[5]);
	outputState.LPCh7.setCurrent(lpChannelCurrent[6]);
	outputState.LPCh8.setCurrent(lpChannelCurrent[7]);
	outputState.LPCh9.setCurrent(lpChannelCurrent[8]);
	outputState.LPCh10.setCurrent(lpChannelCurrent[9]);

	outputState.HPCh1.setCurrent(hpChannelCurrent[0]);
	outputState.HPCh2.setCurrent(hpChannelCurrent[0]);
	outputState.HPCh3.setCurrent(hpChannelCurrent[0]);
	outputState.HPCh4.setCurrent(hpChannelCurrent[0]);

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
	msgCurrent1.set<signals::PDU_LPCh1_Current> (lpChannelCurrent[0]);
	msgCurrent1.set<signals::PDU_LPCh2_Current> (lpChannelCurrent[1]);
	msgCurrent1.set<signals::PDU_LPCh3_Current> (lpChannelCurrent[2]);
	msgCurrent1.set<signals::PDU_LPCh4_Current> (lpChannelCurrent[3]);
	msgCurrent1.set<signals::PDU_LPCh5_Current> (lpChannelCurrent[4]);
	msgCurrent1.send();

	osDelay(pdMS_TO_TICKS(1));	// TODO when cz_interface is fixed, this can be removed

	// Current of LPchannels 6 to 10
	Message <messages::PDU_TX_LP_Current2> msgCurrent2;
	msgCurrent2.set<signals::PDU_LPCh6_Current> (lpChannelCurrent[5]);
	msgCurrent2.set<signals::PDU_LPCh7_Current> (lpChannelCurrent[6]);
	msgCurrent2.set<signals::PDU_LPCh8_Current> (lpChannelCurrent[7]);
	msgCurrent2.set<signals::PDU_LPCh9_Current> (lpChannelCurrent[8]);
	msgCurrent2.set<signals::PDU_LPCh10_Current> (lpChannelCurrent[9]);
	msgCurrent2.send();

	// Current of HPchannels 1 to 4
	Message <messages::PDU_TX_HP_Current> msgCurrent3;
	msgCurrent3.set<signals::PDU_HPCh1_Current> (hpChannelCurrent[0]);
	msgCurrent3.set<signals::PDU_HPCh2_Current> (hpChannelCurrent[1]);
	msgCurrent3.set<signals::PDU_HPCh3_Current> (hpChannelCurrent[2]);
	msgCurrent3.set<signals::PDU_HPCh4_Current> (hpChannelCurrent[3]);
	msgCurrent3.send();

	// LV battery data
	Message<messages::PDU_TX_LV_BMS> msgBattery;
	msgBattery.set<signals::PDU_LV_Voltage>(OD_InputVoltage_get());
	msgBattery.set<signals::PDU_LV_Current>(lvCurrent);
	msgBattery.set<signals::PDU_LV_SOC>(lvSoc);
	msgBattery.send();

	if(OD_CoolingPumpEnabled != can::signals::PDU_OD_CoolingPumpEnabled::ENABLE){
					outputState.HPCh2.set(false);
	}

	// Short Circuit Debug Message
	Message<messages::PDU_TX_LP_Short_Circuit_Debug> msgDebug1;
	msgDebug1.set<signals::PDU_LPCh1_ShortCnt>(outputState.LPCh1.numShorts());
	msgDebug1.set<signals::PDU_LPCh1_State>(outputState.LPCh1.getStatus());
	msgDebug1.set<signals::PDU_LPCh2_ShortCnt>(outputState.LPCh2.numShorts());
	msgDebug1.set<signals::PDU_LPCh2_State>(outputState.LPCh2.getStatus());
	msgDebug1.set<signals::PDU_LPCh3_ShortCnt>(outputState.LPCh3.numShorts());
	msgDebug1.set<signals::PDU_LPCh3_State>(outputState.LPCh3.getStatus());
	msgDebug1.set<signals::PDU_LPCh4_ShortCnt>(outputState.LPCh4.numShorts());
	msgDebug1.set<signals::PDU_LPCh4_State>(outputState.LPCh4.getStatus());
	msgDebug1.set<signals::PDU_LPCh5_ShortCnt>(outputState.LPCh5.numShorts());
	msgDebug1.set<signals::PDU_LPCh5_State>(outputState.LPCh5.getStatus());
	msgDebug1.set<signals::PDU_LPCh6_ShortCnt>(outputState.LPCh6.numShorts());
	msgDebug1.set<signals::PDU_LPCh6_State>(outputState.LPCh6.getStatus());
	msgDebug1.set<signals::PDU_LPCh7_ShortCnt>(outputState.LPCh7.numShorts());
	msgDebug1.set<signals::PDU_LPCh7_State>(outputState.LPCh7.getStatus());
	msgDebug1.set<signals::PDU_LPCh8_ShortCnt>(outputState.LPCh8.numShorts());
	msgDebug1.set<signals::PDU_LPCh8_State>(outputState.LPCh8.getStatus());
	msgDebug1.set<signals::PDU_LPCh9_ShortCnt>(outputState.LPCh9.numShorts());
	msgDebug1.set<signals::PDU_LPCh9_State>(outputState.LPCh9.getStatus());
	msgDebug1.set<signals::PDU_LPCh10_ShortCnt>(outputState.LPCh10.numShorts());
	msgDebug1.set<signals::PDU_LPCh10_State>(outputState.LPCh10.getStatus());
	msgDebug1.send();

	Message<messages::PDU_TX_HP_Short_Circuit_Debug> msgDebug2;
	msgDebug2.set<signals::PDU_HPCh1_ShortCnt>(outputState.HPCh1.numShorts());
	msgDebug2.set<signals::PDU_HPCh1_State>(outputState.HPCh1.getStatus());
	msgDebug2.set<signals::PDU_HPCh2_ShortCnt>(outputState.HPCh2.numShorts());
	msgDebug2.set<signals::PDU_HPCh2_State>(outputState.HPCh2.getStatus());
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
	if(OD_InputVoltage_get() < OD_batterVoltageLow_get()) {
		WARN_batterVoltageLow_set();
	} else {
		WARN_batterVoltageLow_clear();
	}

	//Warning when battery temperature rises too high
	if(batteryTemperature > OD_overTempWarn_get()) {
		WARN_batterTempHigh_set();
	} else {
		WARN_batterTempHigh_clear();
	}

	// Error and shutdown when LV battery voltage gets critical low
	if(OD_InputVoltage_get() < OD_batterVoltageCritical_get() || errorUndervoltageCounter >= 200) {

		errorUndervoltageCounter++;

		// If error is present for 100 or more cycles (500ms), set the error
		if(errorUndervoltageCounter == 100) {
			//ERR_batterVoltageCritical_set();

			// If undervoltage is longer than 1s, shut everything down
		} else if(errorUndervoltageCounter >= 200) {
			outputState.LPCh1.set(false);
			outputState.LPCh2.set(false);
			outputState.LPCh3.set(false);
			outputState.LPCh4.set(false);
			outputState.LPCh5.set(false);
			outputState.LPCh6.set(false);
			outputState.LPCh7.set(false);
			outputState.LPCh8.set(false);
			outputState.LPCh9.set(false);
			outputState.LPCh10.set(false);
			outputState.HPCh1.set(false);
			outputState.HPCh2.set(false);
			outputState.HPCh3.set(false);
			outputState.HPCh4.set(false);
			//outputState.D1.set(false);
			//outputState.D2.set(false);
			//outputState.D3.set(false);
			//outputState.D4.set(false);
			//outputState.SDC.set(false);
		}
	} else {
		errorUndervoltageCounter = 0;
	}


	if(batteryTemperature > OD_overTempCritical_get() || errorOvertemperatureCounter >= 400) {
		errorOvertemperatureCounter++;

		// If error is present for 200 or more cycles (1000ms), set the error
		if(errorOvertemperatureCounter == 200) {
			//ERR_batterTempCritical_set();

			// If overtemperature is longer than 2s, shut everything down
		} else if(errorOvertemperatureCounter >= 400) {
			outputState.LPCh1.set(false);
			outputState.LPCh2.set(false);
			outputState.LPCh3.set(false);
			outputState.LPCh4.set(false);
			outputState.LPCh5.set(false);
			outputState.LPCh6.set(false);
			outputState.LPCh7.set(false);
			outputState.LPCh8.set(false);
			outputState.LPCh9.set(false);
			outputState.LPCh10.set(false);
			outputState.HPCh1.set(false);
			outputState.HPCh2.set(false);
			outputState.HPCh3.set(false);
			outputState.HPCh4.set(false);
			//outputState.D1.set(false);
			//outputState.D2.set(false);
			//outputState.D3.set(false);
			//outputState.D4.set(false);
			outputState.SDC.set(false);
		}
	} else {
		errorOvertemperatureCounter = 0;
	}

	// Error and shutdown when LV battery current exceeds critical value
	if(lvCurrent > OD_batteryOvercurrent_get() || errorOvercurrentCounter > 10) {

		errorOvercurrentCounter++;

		// If error is present for 10 or more cycles, shut down pod and set the error
		if(errorOvercurrentCounter > 10) {
			//ERR_batteryOvercurrent_set();
			outputState.LPCh1.set(false);
			outputState.LPCh2.set(false);
			outputState.LPCh3.set(false);
			outputState.LPCh4.set(false);
			outputState.LPCh5.set(false);
			outputState.LPCh6.set(false);
			outputState.LPCh7.set(false);
			outputState.LPCh8.set(false);
			outputState.LPCh9.set(false);
			outputState.LPCh10.set(false);
			outputState.HPCh1.set(false);
			outputState.HPCh2.set(false);
			outputState.HPCh3.set(false);
			outputState.HPCh4.set(false);
			//outputState.D1.set(false);
			//outputState.D2.set(false);
			//outputState.D3.set(false);
			//outputState.D4.set(false);
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
	outputState.LPCh1.update();
	outputState.LPCh2.update();
	outputState.LPCh3.update();
	outputState.LPCh4.update();
	outputState.LPCh5.update();
	outputState.LPCh6.update();
	outputState.LPCh7.update();
	outputState.LPCh8.update();
	outputState.LPCh9.update();
	outputState.LPCh10.update();
	outputState.HPCh1.update();
	outputState.HPCh2.update();
	outputState.HPCh3.update();
	outputState.HPCh4.update();
	/*
	outputState.D1.update();
	outputState.D2.update();
	outputState.D3.update();
	outputState.D4.update();
	*/


	// Standard output channels with PWM support

	//Somehow all timers except of TIM2 can't save dutycycles of 100% so that the +1 was deleted.
	if(state::get() == state::STATE::POD_OFF){
		// LPCh1 is TIM12_CH2
		htim12.Instance->CCR2 = 0;

		// LPCh2 is TIM2_CH3
		htim2.Instance->CCR3 = 0;

		// LPCh3 is TIM2_CH1
		htim2.Instance->CCR1 = 0;

		// LPCh8 is TIM8_CH1
		htim8.Instance->CCR1 = 0;

		// LPCh9 is TIM4_CH2
		htim4.Instance->CCR2 = 0;

		// LPCh10 is TIM11_CH1
		htim11.Instance->CCR1 = 0;

		// HPCh1 is TIM8_CH4
		htim8.Instance->CCR4 = 0;

		// HPCh2 is TIM8_CH2
		htim8.Instance->CCR2 = 0;

		// D1 is TIM3_CH1 (is controlled by ProjectXX.hpp)
		//htim3.Instance->CCR1= 0;

		// D2 is TIM10_CH1
		htim10.Instance->CCR1 = 0;

		// D3 is TIM2_CH4
		htim2.Instance->CCR4 = 0;

		// D4 is TIM3_CH3
		htim3.Instance->CCR3 = 0;

		// Standard On/Off output
		HAL_GPIO_WritePin(LP4_control_GPIO_Port, LP4_control_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LP5_control_GPIO_Port, LP5_control_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LP6_control_GPIO_Port, LP6_control_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LP7_control_GPIO_Port, LP7_control_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HP3_control_GPIO_Port, HP3_control_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HP4_control_GPIO_Port, HP4_control_Pin, GPIO_PIN_RESET);

		// SDC switch
		HAL_GPIO_WritePin(SDC_control_GPIO_Port, SDC_control_Pin, GPIO_PIN_RESET);
	}else{
		// LPCh1 is TIM12_CH2
		if(outputState.LPCh1.getSwitch()) {
			htim12.Instance->CCR2 = outputState.LPCh1.getDuty() * (htim12.Instance->ARR) / 100.0f;
		} else {
			htim12.Instance->CCR2 = 0;
		}

		// LPCh2 is TIM2_CH3
		if(outputState.LPCh2.getSwitch()) {
			htim2.Instance->CCR3 = outputState.LPCh2.getDuty() * (htim2.Instance->ARR + 1) / 100.0f;
		} else {
			htim2.Instance->CCR3 = 0;
		}

		// LPCh3 is TIM2_CH1
		if(outputState.LPCh3.getSwitch()) {
			htim2.Instance->CCR1 = outputState.LPCh3.getDuty() * (htim2.Instance->ARR + 1) / 100.0f;
		} else {
			htim2.Instance->CCR1 = 0;
		}

		// LPCh8 is TIM8_CH1
		if(outputState.LPCh8.getSwitch()) {
			htim8.Instance->CCR1 = outputState.LPCh3.getDuty() * (htim8.Instance->ARR ) / 100.0f;
		} else {
			htim8.Instance->CCR1 = 0;
		}

		// LPCh9 is TIM4_CH2
		if(outputState.LPCh9.getSwitch()) {
			htim4.Instance->CCR2 = outputState.LPCh9.getDuty() * (htim4.Instance->ARR) / 100.0f;
		} else {
			htim4.Instance->CCR2 = 0;
		}

		// LPCh10 is TIM11_CH1
		if(outputState.LPCh10.getSwitch()) {
			htim11.Instance->CCR1 = outputState.LPCh10.getDuty() * (htim11.Instance->ARR) / 100.0f;
		} else {
			htim11.Instance->CCR1 = 0;
		}

		// HPCh1 is TIM8_CH4
		if(outputState.HPCh1.getSwitch()) {
			htim8.Instance->CCR4 = outputState.HPCh1.getDuty() * (htim8.Instance->ARR) / 100.0f;
		} else {
			htim8.Instance->CCR4 = 0;
		}

		// HPCh2 is TIM8_CH2
		if(outputState.HPCh2.getSwitch()) {
			htim8.Instance->CCR2 = outputState.HPCh2.getDuty() * (htim8.Instance->ARR) / 100.0f;
		} else {
			htim8.Instance->CCR2 = 0;
		}

		// D1 is TIM3_CH1 (is controlled by ProjectXX.hpp)
		/*
		if(outputState.D1.getSwitch()) {
			htim3.Instance->CCR1 = outputState.D1.getDuty() * (htim3.Instance->ARR) / 100.0f;
		} else {
			htim3.Instance->CCR1= 0;
		}
		*/

		// D2 is TIM10_CH1
		if(outputState.D2.getSwitch()) {
			htim10.Instance->CCR1 = outputState.D2.getDuty() * (htim10.Instance->ARR) / 100.0f;
		} else {
			htim10.Instance->CCR1 = 0;
		}

		// D3 is TIM2_CH4
		if(outputState.D3.getSwitch()) {
			htim2.Instance->CCR4 = outputState.D3.getDuty() * (htim2.Instance->ARR) / 100.0f;
		} else {
			htim2.Instance->CCR4 = 0;
		}

		// D4 is TIM3_CH3
		if(outputState.D4.getSwitch()) {
			htim3.Instance->CCR3 = outputState.D4.getDuty() * (htim3.Instance->ARR) / 100.0f;
		} else {
			htim3.Instance->CCR3 = 0;
		}

		// Standard On/Off output
		HAL_GPIO_WritePin(LP4_control_GPIO_Port, LP4_control_Pin, outputState.LPCh4.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LP5_control_GPIO_Port, LP5_control_Pin, outputState.LPCh5.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LP6_control_GPIO_Port, LP6_control_Pin, outputState.LPCh6.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LP7_control_GPIO_Port, LP7_control_Pin, outputState.LPCh7.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HP3_control_GPIO_Port, HP3_control_Pin, outputState.HPCh3.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(HP4_control_GPIO_Port, HP4_control_Pin, outputState.HPCh4.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);

		// SDC switch
		HAL_GPIO_WritePin(SDC_control_GPIO_Port, SDC_control_Pin, outputState.SDC.getSwitch() ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

/**
 * Send out CAN messages
 */
void sendData() {
	// PDU status
	Message<messages::PDU_TX_Status> msgStatus;
	msgStatus.set<signals::PDU_TX_Enabled>(pduEnabled);
	msgStatus.set<signals::PDU_TX_PEHWEnabled>(false);
	msgStatus.set<signals::PDU_TX_ErrorFlag>(anyErrorPresent());
	msgStatus.send();
}


uint32_t lastDataRead = 0;
uint32_t lastStatusSent = 0;

// Main Task of the PDU
static void pduAppFunction(void *pvArguments) {

	LED_RGB_Write(0, 0, 0);

	// Wait 100ms to be sure that battery voltage was already read at least once by the TaskManager.cpp
	osDelay(pdMS_TO_TICKS(100));

	adc2.init(50);	// Read each channel 50 times and average
	osDelay(pdMS_TO_TICKS(25));	// Wait 25ms to ensure that ADC is initialized

	// Start timers for PWM generation
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);	// LPCh1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// LPCh2
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// LPCh3
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);	// LPCh8
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);	// LPCh9
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);	// LPCh10
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);	// HPCh1
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);	// HPCh2
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	// D1
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);	// D2
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);	// D3
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	// D4

	// Initial read of data
	readAndSendData();

	// Reset watchdog
	stateMachineWatchdog.reset();

	// Let the state machine know that we finished the ECU setup and are ready for operation
	cz_interface::getInstance()->setStatus(pre_operational);

	while (1) {
		// Blink the status LEDs
		blinkStatusLed();

		// Receive control messages
		receiveCanMessages();

		// Read channel currents, send the CAN messages and update the cooling PWM
		if(xTaskGetTickCount() - lastDataRead >= OD_currentReadInterval_get()) {
			lastDataRead = xTaskGetTickCount();

			// Read the current of all channels and send it
			readAndSendData();
		}

		// Error monitoring
		/*
		if(pduEnabled == false) {
			batterySafetyChecks();
		}

		if(pduEnabled == true) {
			batterySafetyChecks();
			if(stateMachineWatchdog.isTimedOut()) {
				//ERR_watchdogStateMachine_set();
			}
		}
		*/

		// If any error is present, shutdown PeHwEnable and SDC and enable red led
		if(anyErrorPresent()) {
			LED_Red_Write(255);
		} else {
			LED_Red_Write(0);
		}

		if(outputState.SDC.get()) {
			LED_Orange_Write(255);
		} else {
			LED_Orange_Write(0);
		}


		// Update channels: on/off and duty cycle
		updateChannels();

		// Send out status message with configured interval or when there is a change in PDU status
		if(xTaskGetTickCount() - lastStatusSent >= OD_statusSendInterval_get() ||
				lastPduEnabled != pduEnabled || lastPeHwEnabled != outputState.LPCh6.get() || lastErrorFlag != anyErrorPresent()) {

			lastPduEnabled = pduEnabled;
			lastPeHwEnabled = outputState.LPCh6.get();
			lastErrorFlag = anyErrorPresent();

			lastStatusSent = xTaskGetTickCount();

			// Send out data to CAN bus
			sendData();
		}

		// Delay until next check
		osDelay(pdMS_TO_TICKS(50));
	}
}


#endif /* INCLUDE_PDU_HPP_ */
