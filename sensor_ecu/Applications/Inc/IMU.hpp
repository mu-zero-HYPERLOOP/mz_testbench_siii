/*
 * IMU.hpp
 *
 *  Created on: 20 Apr 2022
 *      Author: piril
 *      #ifndef INCLUDE_IMU_HPP_  #define INCLUDE_IMU_HPP_
 *       #endif  INCLUDE_IMU_HPP_
 *
 *
 */

#ifndef IMU_HPP_
#define IMU_HPP_
#include "Application.hpp"

class IMU : public Application {
public:
	IMU();
	virtual ~IMU();

	static void receiveData(void*);
	static void updatePosition(void*);
};

#endif /* IMU_HPP_ */
