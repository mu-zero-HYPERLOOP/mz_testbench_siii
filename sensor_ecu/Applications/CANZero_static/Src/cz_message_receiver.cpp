/*
 * cz_message_receiver.cpp
 *
 *  Created on: Apr 23, 2023
 *      Author: OfficeLaptop
 */
#include "cz_message_receiver.hpp"
#include <cinttypes>


namespace canzero {

struct receiver_t {
	std::function<void(RxMessage&)> m_callback;
	uint32_t m_id;
	bool m_extendedId;
};

static receiver_t g_receivers[MAX_REGISTERD_RECEIVERS];
uint32_t g_size = 0;

void registerMessageReceiverInternal(std::function<void(RxMessage&)> receiver, uint32_t msgId, bool extendedId){
	const receiver_t recv = {
		.m_callback = receiver,
		.m_id = msgId,
		.m_extendedId = extendedId
	};
	g_receivers[g_size++] = recv;
}

bool processRxMessageReceiver(RxMessage& message){
	if(message.rxHeader.IDE == CAN_ID_STD){
		for(uint32_t i = 0;i<g_size;i++){
			if((g_receivers[i].m_extendedId == false) && (message.rxHeader.StdId == g_receivers[i].m_id)){
				g_receivers[i].m_callback(message);
				return true;
			}
		}
	}else{
		for(uint32_t i = 0;i<g_size;i++){
			if((g_receivers[i].m_extendedId == true) && (message.rxHeader.ExtId == g_receivers[i].m_id)){
				g_receivers[i].m_callback(message);
				return true;
			}
		}
	}
	return false;
}

}



