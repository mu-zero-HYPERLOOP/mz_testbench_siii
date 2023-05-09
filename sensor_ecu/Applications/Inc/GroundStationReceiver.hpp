/*
 * GroupStationReceiver.hpp
 *
 *  Created on: May 4, 2023
 *      Author: OfficeLaptop
 */

#pragma once

#include "canzero.hpp"
#include <cinttypes>

enum GroundStationCommand : uint8_t{
	COMMAND_NONE = 0,
	COMMAND_ENTER_LAUNCH_PREP = 1,
	COMMAND_LAUNCH = 2,
	COMMAND_STOP = 3,
	COMMAND_IDLE = 4,
	COMMAND_RESET = 5
};

class GroundStationReceiver {
public:

	static GroundStationReceiver& getInstance(){
		static GroundStationReceiver instance;
		return instance;
	}

	~GroundStationReceiver(){
		can::unregisterMessageReceiver(m_receiverId);
	}
	GroundStationReceiver(GroundStationReceiver&) = delete;
	GroundStationReceiver(GroundStationReceiver&&) = delete;
	GroundStationReceiver& operator=(GroundStationReceiver&) = delete;
	GroundStationReceiver& operator=(GroundStationReceiver&&) = delete;

	[[nodiscard]] inline GroundStationCommand getLastCommand(){
		return m_lastCommand;
	}
private:
	explicit GroundStationReceiver(){
		m_receiverId = can::registerMessageReceiver<can::messages::TEST_GROUND_STATION_CONTROLL>([this](RxMessage& msg){
			this->callback(msg);
		});
	}

	void callback(RxMessage& raw){
		can::Message<can::messages::TEST_GROUND_STATION_CONTROLL> msg {raw};
		m_lastCommand = static_cast<GroundStationCommand>(msg.get<can::signals::TEST_GROUND_STATION_COMMAND>());
	}
	unsigned int m_receiverId;
	GroundStationCommand m_lastCommand = COMMAND_NONE;

};
