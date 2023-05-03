/*
 * Event.hpp
 *
 *  Created on: Apr 16, 2023
 *      Author: karl
 */

#ifndef INC_EVENT_HPP_
#define INC_EVENT_HPP_

#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"

template<typename T, size_t CAPACITY>
class EventQueue {
public:
	explicit EventQueue(){
		m_semaphore = osSemaphoreNew(CAPACITY, 0, NULL);
	}
	~EventQueue(){
		osSemaphoreDelete(m_semaphore);
	}

	void enqueue(const T& v){
		m_queue[(m_front + osSemaphoreGetCount(m_semaphore) ) % CAPACITY] = v;
		osSemaphoreRelease(m_semaphore);
	}

	bool empty(){
		return osSemaphoreGetCount(m_semaphore) == 0;
	}

	const T& pop(){
		const T& v = m_queue[m_front];
		m_front = (m_front + 1) % CAPACITY;
		return v;
	}

	const T& await(){
		osSemaphoreAcquire(m_semaphore, HAL_MAX_DELAY);
		return pop();
	}

	const T& peek(){
		return m_queue[m_front];
	}

private:
	osSemaphoreId_t m_semaphore;
	T m_queue[CAPACITY];
	size_t m_front = 0;
};

template<size_t CAPACITY>
class EventVoidQueue {
public:
	explicit EventVoidQueue(){
		m_semaphore = osSemaphoreNew(CAPACITY, 0, NULL);
	}
	~EventVoidQueue(){
		osSemaphoreDelete(m_semaphore);
	}

	void enqueue(){
		osSemaphoreRelease(m_semaphore);
	}

	bool empty(){
		return osSemaphoreGetCount(m_semaphore) == 0;
	}

	void await(){
		osSemaphoreAcquire(m_semaphore, HAL_MAX_DELAY);
	}

private:
	osSemaphoreId_t m_semaphore;
	size_t m_front = 0;
};



#endif /* INC_EVENT_HPP_ */
