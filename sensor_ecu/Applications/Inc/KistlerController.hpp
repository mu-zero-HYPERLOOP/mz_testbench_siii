/*
 * KistlerController.hpp
 *
 *  Created on: May 3, 2023
 *      Author: OfficeLaptop
 */

#pragma once
#include "canzero.hpp"

class KistlerController {
public:
	explicit KistlerController(){
		m_receiverId = can::registerMessageReceiver<can::messages::OpticalSensor_TX_MainData>([this](RxMessage& msg){
			this->callback(msg);
		});
	}
	~KistlerController(){
		can::unregisterMessageReceiver(m_receiverId);
	}
	KistlerController(KistlerController&) = delete;
	KistlerController(KistlerController&&) = delete;
	KistlerController& operator=(KistlerController&) = delete;
	KistlerController& operator=(KistlerController&&) = delete;

	[[nodiscard]] inline float getVelocity(){
		return m_velocity;
	}

	[[nodiscard]] inline float getPosition(){
		return m_position;
	}

private:

	void callback(RxMessage& raw){
		can::Message<can::messages::OpticalSensor_TX_MainData> msg {raw};
		m_velocity = msg.get<can::signals::OpticalSensor_TX_Vel>();
		m_position = msg.get<can::signals::OpticalSensor_TX_Distance>();
	}

	unsigned int m_receiverId;
	float m_velocity = -100.0;
	float m_position = -100.0;

};
