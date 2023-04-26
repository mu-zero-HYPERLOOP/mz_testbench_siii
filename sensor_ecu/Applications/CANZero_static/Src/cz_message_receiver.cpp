/*
 * cz_message_receiver.cpp
 *
 *  Created on: Apr 23, 2023
 *      Author: OfficeLaptop
 */
#include "cz_message_receiver.hpp"
#include <cinttypes>
#include "cmsis_os.h"
#include "estdio.hpp"


namespace canzero {

static unsigned int receiverIdAcc = 0;

struct receiver_t {
	std::function<void(RxMessage&)> m_callback;
	uint32_t m_id;
	bool m_extendedId;
	unsigned int m_receiverId;
};

static receiver_t g_receivers[MAX_REGISTERD_RECEIVERS];
uint32_t g_size = 0;

unsigned int registerMessageReceiverInternal(std::function<void(RxMessage&)> receiver, uint32_t msgId, bool extendedId){
	printf("registered Message receiver for id = %ul\n", msgId);
	taskENTER_CRITICAL();
	const unsigned int id = receiverIdAcc++;
	const receiver_t recv = {
		.m_callback = receiver,
		.m_id = msgId,
		.m_extendedId = extendedId,
		.m_receiverId = id
	};
	g_receivers[g_size++] = recv;
	taskEXIT_CRITICAL();
	return id;
}

bool processRxMessageReceiver(RxMessage& message){
	bool foundReceiver = false;
	if(message.rxHeader.IDE == CAN_ID_STD){
		for(uint32_t i = 0;i<g_size;i++){
			if((g_receivers[i].m_extendedId == false) && (message.rxHeader.StdId == g_receivers[i].m_id)){
				g_receivers[i].m_callback(message);
				foundReceiver = true;
			}
		}
	}else{
		for(uint32_t i = 0;i<g_size;i++){
			if((g_receivers[i].m_extendedId == true) && (message.rxHeader.ExtId == g_receivers[i].m_id)){
				g_receivers[i].m_callback(message);
				foundReceiver = true;
			}
		}
	}
	return foundReceiver;
}

}



namespace can {

void unregisterMessageReceiver(unsigned int id){
	using namespace canzero;
	taskENTER_CRITICAL();
	for(size_t i = 0;i<g_size;i++){
		if(g_receivers[i].m_receiverId == id){
			g_receivers[i] = g_receivers[--g_size];
		}
	}
	taskEXIT_CRITICAL();
}

}

