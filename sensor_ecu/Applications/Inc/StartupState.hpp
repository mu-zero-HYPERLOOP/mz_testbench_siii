/*
 * StartupState.hpp
 *
 *  Created on: Apr 25, 2023
 *      Author: OfficeLaptop
 */
#pragma once

#include "State.hpp"
#include "peripheral_config.hpp"
#include "canzero.hpp"
#include "estdio.hpp"
#include "FiducialSensor.hpp"

class StartupState: public State {
public:
	void setup() {
		printf("enter startup state\n");
		m_readyReceiverId = can::registerMessageReceiver<
				can::messages::SENSOR_Ready>([&](RxMessage &msg) {
			onMessageReady(msg);
		});
		m_helloWorldReceiverId = can::registerMessageReceiver<
				can::messages::SENSOR_HELLO_WORLD>([&](RxMessage &msg) {
			onMessageHelloWorld(msg);
		});
	}

	void update() {
	}

	void dispose() {
		printf("exit startup state\n");
		can::unregisterMessageReceiver(m_readyReceiverId);
		can::unregisterMessageReceiver(m_helloWorldReceiverId);
	}

private:
	void onMessageReady(RxMessage &msg) {
		printf("Received Ready\n");
	}
	void onMessageHelloWorld(RxMessage &msg) {
		printf("Received HelloWorld!");
	}

	FiducialSensor m_fiducialRight = FiducialSensor(
			g_peripherals.m_fiducialRightConfig);
	FiducialSensor m_fiducialLeft = FiducialSensor(
			g_peripherals.m_fiducialLeftConfig);
	unsigned int m_readyReceiverId;
	unsigned int m_helloWorldReceiverId;
};

