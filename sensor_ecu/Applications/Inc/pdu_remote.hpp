/*
 * pdu_control.hpp
 *
 *  Created on: 11 May 2023
 *      Author: karl
 */

#pragma once
#include <cinttypes>

namespace pdu {

enum LpChannel : uint16_t{
	LP_CHANNEL1 = 0,
	LP_CHANNEL2,
	LP_CHANNEL3,
	LP_CHANNEL4,
	LP_CHANNEL5,
	LP_CHANNEL6,
	LP_CHANNEL7,
	LP_CHANNEL8,
	LP_CHANNEL9,
	LP_CHANNEL10,
};

enum HpChannel : uint16_t{
	HP_CHANNEL1 = 0,
	HP_CHANNEL2 = 1,
	HP_CHANNEL3 = 2,
	HP_CHANNEL4 = 3
};

enum ChannelStatus {
	CHANNEL_STATUS_OFF,
	CHANNEL_STATUS_PENDING_OFF,
	CHANNEL_STATUS_ON,
	CHANNEL_STATUS_PENDING_ON
};

void enableChannel(HpChannel channel);

void disableChannel(HpChannel channel);

void enableChannel(LpChannel channel);

void disableChannel(LpChannel channel);

void killMe();

ChannelStatus getChannelStatus(LpChannel channel);
ChannelStatus getChannelStatus(HpChannel channel);


void init(bool minimizeMessages = true);

void update();

}
