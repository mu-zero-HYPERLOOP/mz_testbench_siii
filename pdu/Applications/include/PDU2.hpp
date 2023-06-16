/*
 * PDU.hpp
 *
 *  Created on:
 *      Author:
 *
 */

#ifndef INCLUDE_PDU2_HPP_
#define INCLUDE_PDU2_HPP_

#include "main.h"
#include "cz_interface.hpp"
#include "dbc_parser.hpp"
#include "Watchdog.hpp"
#include <cmath>
#include <algorithm>
#include "estdio.hpp"

//AdcDma<4> adc1 { &hadc1 }; //ADC class to read basic BCU signals, already defined in TaskManager.cpp
AdcDma<14> adc2 { &hadc2 };	// ADC class to read the current of the channels

bool m_enable;

bool m_lpChannels[10] = {
		false,
		false,
		false,
		false,
		false,
		false,
		false,
		false,
		false,
		false

};
bool m_hpChannels[10];
bool m_sdc = true;

void receiveCAN(){
	using namespace can;
	RxMessage rxRawMsg;
	while(xMessageBufferReceive(handlePduRxMessageBuffer, &rxRawMsg, sizeof(rxRawMsg), 0) != 0) {
		// Control message from state machine
		if(checkRxMessage<messages::PDU_RX_Control>(rxRawMsg)) {
			can::Message<messages::PDU_RX_Control> msg{rxRawMsg};
			m_enable = msg.get<can::signals::PDU_RX_Enable>();

		}
		else if(checkRxMessage<messages::PDU_RX_LP_Dutycycle>(rxRawMsg)) {	// Duty cycle message for manual control
			can::Message<messages::PDU_RX_LP_Dutycycle> dutyMsg{rxRawMsg};

			float lpch1_duty = dutyMsg.get<signals::PDU_LPCh1_Dutycycle>();
			m_lpChannels[0] = lpch1_duty != 0;
			float lpch2_duty = dutyMsg.get<signals::PDU_LPCh2_Dutycycle>();
			m_lpChannels[1] = lpch2_duty != 0;
			float lpch3_duty = dutyMsg.get<signals::PDU_LPCh3_Dutycycle>();
			m_lpChannels[2] = lpch3_duty != 0;
			float lpch8_duty = dutyMsg.get<signals::PDU_LPCh8_Dutycycle>();
			m_lpChannels[7] = lpch8_duty != 0;
			float lpch9_duty = dutyMsg.get<signals::PDU_LPCh9_Dutycycle>();
			m_lpChannels[8] = lpch9_duty != 0;
			float lpch10_duty = dutyMsg.get<signals::PDU_LPCh10_Dutycycle>();
			m_lpChannels[9] = lpch10_duty != 0;

		} else if(checkRxMessage<messages::PDU_RX_HP_D_Dutycycle>(rxRawMsg)) {	// Duty cycle message for manual control
			can::Message<messages::PDU_RX_HP_D_Dutycycle> dutyMsg{rxRawMsg};

			float hpch1_duty = dutyMsg.get<signals::PDU_HPCh1_Dutycycle>();
			m_hpChannels[0] = hpch1_duty != 0;
			float hpch2_duty = dutyMsg.get<signals::PDU_HPCh2_Dutycycle>();
			m_hpChannels[1] = hpch2_duty != 0;
		} else if(checkRxMessage<messages::PDU_RX_LP_Enable>(rxRawMsg)){
			can::Message<messages::PDU_RX_LP_Enable> lpEnableMsg {rxRawMsg};

			m_lpChannels[3] = (lpEnableMsg.get<signals::PDU_RX_LPCh4_Enable>());
			m_lpChannels[4] = (lpEnableMsg.get<signals::PDU_RX_LPCh5_Enable>());
			m_lpChannels[5] = (lpEnableMsg.get<signals::PDU_RX_LPCh6_Enable>());
			m_lpChannels[6] = (lpEnableMsg.get<signals::PDU_RX_LPCh7_Enable>());

		}
	}
}

static void updateLpChannels(){

	if(m_lpChannels[0]) {
		htim12.Instance->CCR2 = (htim12.Instance->ARR); //full duty.
	} else {
		htim12.Instance->CCR2 = 0;
	}

	// LPCh2 is TIM2_CH3
	if(m_lpChannels[1]) {
		htim2.Instance->CCR3 = (htim2.Instance->ARR + 1);
	} else {
		htim2.Instance->CCR3 = 0;
	}

	// LPCh3 is TIM2_CH1
	if(m_lpChannels[2]) {
		htim2.Instance->CCR1 = (htim2.Instance->ARR + 1);
	} else {
		htim2.Instance->CCR1 = 0;
	}

	// LPCh8 is TIM8_CH1
	if(m_lpChannels[7]) {
		htim8.Instance->CCR1 = (htim8.Instance->ARR );
	} else {
		htim8.Instance->CCR1 = 0;
	}

	// LPCh9 is TIM4_CH2
	if(m_lpChannels[8]) {
		htim4.Instance->CCR2 = (htim4.Instance->ARR);
	} else {
		htim4.Instance->CCR2 = 0;
	}

	// LPCh10 is TIM11_CH1
	if(m_lpChannels[9]) {
		htim11.Instance->CCR1 = (htim11.Instance->ARR);
	} else {
		htim11.Instance->CCR1 = 0;
	}

	// Standard On/Off output
	HAL_GPIO_WritePin(LP4_control_GPIO_Port, LP4_control_Pin, m_lpChannels[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LP5_control_GPIO_Port, LP5_control_Pin, m_lpChannels[4] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LP6_control_GPIO_Port, LP6_control_Pin, m_lpChannels[5] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LP7_control_GPIO_Port, LP7_control_Pin, m_lpChannels[7] ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// SDC switch
	HAL_GPIO_WritePin(SDC_control_GPIO_Port, SDC_control_Pin, m_sdc ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void updateHpChannels(){
	// HPCh1 is TIM8_CH4
	if(m_hpChannels[0]) {
		htim8.Instance->CCR4 = (htim8.Instance->ARR);
	} else {
		htim8.Instance->CCR4 = 0;
	}

	// HPCh2 is TIM8_CH2
	if(m_hpChannels[1]) {
		htim8.Instance->CCR2 = (htim8.Instance->ARR);
	} else {
		htim8.Instance->CCR2 = 0;
	}

	HAL_GPIO_WritePin(HP3_control_GPIO_Port, HP3_control_Pin, m_hpChannels[2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HP4_control_GPIO_Port, HP4_control_Pin, m_hpChannels[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void initPWM(){
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
}

static void initAdc(){
	adc2.init(50);	// Read each channel 50 times and average
}

// Main Task of the PDU
static void pduAppFunction(void *pvArguments) {
	initPWM();
	initAdc();


	osDelay(pdMS_TO_TICKS(100));
	if(m_lpChannels[0]) {
		htim12.Instance->CCR2 = (htim12.Instance->ARR); //full duty.
	} else {
		htim12.Instance->CCR2 = 0;
	}

	HAL_GPIO_WritePin(LP5_control_GPIO_Port, LP5_control_Pin,  GPIO_PIN_SET);

	osDelay(osWaitForever);
	while(true){
		receiveCAN();
		updateLpChannels();
		updateHpChannels();

		osDelay(pdMS_TO_TICKS(50));
	}

}


#endif /* INCLUDE_PDU2_HPP_ */
