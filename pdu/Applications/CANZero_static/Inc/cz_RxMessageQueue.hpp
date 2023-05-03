/*
 * cz_FilteredMessageQueue.hpp
 *
 *  Created on: Apr 28, 2023
 *      Author: OfficeLaptop
 */

#ifndef CANZERO_STATIC_INC_CZ_RXMESSAGEQUEUE_HPP_
#define CANZERO_STATIC_INC_CZ_RXMESSAGEQUEUE_HPP_

#include "cz_message_receiver.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "message_buffer.h"
#include "cz_typedefinitions.hpp"
#include <cinttypes>
#include <utility>
#include <limits>
#include "queue.h"

namespace can {

template<typename MESSAGE>
class RxMessageQueue {
private:
	static constexpr unsigned int INVALID_ID = std::numeric_limits<unsigned int>::max();
public:
	RxMessageQueue() {

	}
	~RxMessageQueue() {
		disable();
	}

	void enable(){
		if(m_receiverId == INVALID_ID){
			m_receiverId = registerMessageReceiver([this](RxMessage& msg){
				this->receiveCallback(msg);
			});
		}

	}

	void disable(){
		if(m_receiverId != INVALID_ID){
			unregisterMessageReceiver(m_receiverId);
		}
	}

	inline bool isEmpty(){
		return xMessageBufferIsEmpty(m_messageBuffer) == pdTRUE;
	}

	inline bool isFull(){
		return xMessageBufferIsFull(m_messageBuffer) == pdTRUE;
	}

	inline bool hasAny(){
		return not isEmpty();
	}

	inline can::Message<MESSAGE> dequeue(){
		RxMessage rxMsg;
		xMessageBufferReceive(m_messageBuffer, &rxMsg, sizeof(RxMessage), 0);
		return can::Message<MESSAGE>(std::move(rxMsg));
	}


private:

	inline void receiveCallback(RxMessage &msg) {
		xMessageBufferSend(m_messageBuffer, &msg, sizeof(RxMessage), 10);
	}

	static constexpr size_t MSG_BUFFER_NUM_MESSAGES = 5; // Number of messages to store in the message buffer
	static constexpr size_t MSG_BUFFER_SIZE = MSG_BUFFER_NUM_MESSAGES
			* (sizeof(RxMessage) + 4); // 4 bytes overhead to store the size_t
	MessageBufferHandle_t m_messageBuffer;
	unsigned int m_receiverId = INVALID_ID;

};

}

#endif /* CANZERO_STATIC_INC_CZ_RXMESSAGEQUEUE_HPP_ */
