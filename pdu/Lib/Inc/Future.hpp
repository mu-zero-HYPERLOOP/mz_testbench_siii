/*
 * Future.hpp
 *
 *  Created on: Apr 13, 2023
 *      Author: karl
 */

#ifndef INC_FUTURE_HPP_
#define INC_FUTURE_HPP_

#include <functional>
#include <optional>
#include "cmsis_os.h"

template<typename T> class Future {
private:
	union optional {
	public:
		char null[sizeof(T)];
		T obj;
		optional() : null{} {}
	};
public:
	Future() {
		m_semaphore = osSemaphoreNew(1,0,NULL);
	}
	~Future(){
		osSemaphoreDelete(m_semaphore);

	}
	Future(Future&) = delete;
	Future(Future&&) = delete;
	Future& operator=(Future&) = delete;
	Future& operator=(Future&&) = delete;

	void then(std::function<void(T&)> cplt) {
		if (m_handled)
			return;
		if(m_complete){
			cplt(m_value.obj);
		}else{
			m_completionCallback = cplt;
		}
	}
	void complete(T value) {
		if (m_handled)
			return;
		m_value.obj = value;
		if (m_completionCallback != nullptr) {
			m_completionCallback(m_value.obj);
			m_handled = true;
		}
		m_complete = true;
		osSemaphoreRelease(m_semaphore);
	}
	T get() {
		osSemaphoreAcquire(m_semaphore, HAL_MAX_DELAY);
		m_handled = true;
		return m_value.obj;
	}

	bool isComplete() {
		return m_complete;
	}
	bool isHandled() {
		return m_handled;
	}

	void reset() {
		m_complete = false;
		m_completionCallback = nullptr;
		m_handled = false;
	}

private:
	std::function<void(T&)> m_completionCallback = nullptr;
	optional m_value;
	bool m_complete = false;
	bool m_handled = false;
	osSemaphoreId_t m_semaphore;
};

#endif /* INC_FUTURE_HPP_ */
