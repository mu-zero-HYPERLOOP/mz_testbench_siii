/*
 * status_led_controll.hpp
 *
 *  Created on: May 21, 2023
 *      Author: OfficeLaptop
 */


#include "status_led_controll.hpp"
#include "sensor_ecu_remote.hpp"
#include "State.hpp"
#include "GPIOWriteController.hpp"
#include "gpio.h"

namespace status_led {

static volatile bool m_ledErrorStatus = false;

GPIOWriteController m_errorLed(TMP_LED_OUTPUT_GPIO_Port, TMP_LED_OUTPUT_Pin);

void ledStatusReceiver(RxMessage& raw){
	can::Message<can::messages::CLU_RX_StatusLedControll> msg{raw};
	m_ledErrorStatus = msg.get<can::signals::CLU_RX_ErrorStatus>();
}

void init(){
	can::registerMessageReceiver<can::messages::CLU_RX_StatusLedControll>(ledStatusReceiver);
}

void update(){
	m_errorLed.write(m_ledErrorStatus);

	//TODO implement different leds.
}

}
