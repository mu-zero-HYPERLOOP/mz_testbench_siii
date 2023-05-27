/*
 * led_status.hpp
 *
 *  Created on: May 21, 2023
 *      Author: OfficeLaptop
 */

#include "led_status.hpp"
#include "canzero.hpp"

namespace led_status {

bool m_error = false;
bool m_dirtyError = false;

void setError(){
	if(not m_error){
		m_error = true;
		m_dirtyError = true;
	}
}

void resetError(){
	if(m_error){
		m_error = false;
		m_dirtyError = true;
	}
}

void init(){

}

void update(){
	if(m_dirtyError){
		can::Message<can::messages::CLU_RX_StatusLedControll> msg;
		msg.set<can::signals::CLU_RX_ErrorStatus>(m_error);
		msg.send();
		m_dirtyError = false;
	}
	//TODO implement other status led.
}

}
