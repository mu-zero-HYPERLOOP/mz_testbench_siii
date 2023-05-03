/*
 * cz_message_receiver.hpp
 *
 *  Created on: Apr 23, 2023
 *      Author: OfficeLaptop
 */

#ifndef CANZERO_STATIC_INC_CZ_MESSAGE_RECEIVER_HPP_
#define CANZERO_STATIC_INC_CZ_MESSAGE_RECEIVER_HPP_

#include <cinttypes>
#include <functional>
#include "cz_typedefinitions.hpp"

static constexpr size_t MAX_REGISTERD_RECEIVERS = 50;

namespace canzero {

unsigned int registerMessageReceiverInternal(
		std::function<void(RxMessage&)> receiver, uint32_t msgId,
		bool extendedId);

bool processRxMessageReceiver(RxMessage &message);

}

namespace can {

template<typename MESSAGE>
inline unsigned int registerMessageReceiver(
		std::function<void(RxMessage&)> receiver) {
	return canzero::registerMessageReceiverInternal(receiver, MESSAGE::id,
			MESSAGE::isExtendedId);
}

void unregisterMessageReceiver(unsigned int id);

}

#endif /* CANZERO_STATIC_INC_CZ_MESSAGE_RECEIVER_HPP_ */
