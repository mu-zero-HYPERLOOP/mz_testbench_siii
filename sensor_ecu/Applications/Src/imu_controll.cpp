/*
 * imu_controll.hpp
 *
 *  Created on: May 18, 2023
 *      Author: OfficeLaptop
 */

#include <imu.hpp>
#include "ImuMaster.hpp"
#include "canzero.hpp"

namespace imu {


ImuMaster imuMaster;

void init(){
	imuMaster.start();
}

void update(){
		//TODO read sensor data.
		imuMaster.syncRead();
		OD_IMU_AccelX_set(imuMaster.getAccelX());
		OD_IMU_AccelY_set(imuMaster.getAccelY());
		OD_IMU_AccelZ_set(imuMaster.getAccelZ());

		OD_IMU_GyroX_set(imuMaster.getGyroX());
		OD_IMU_GyroY_set(imuMaster.getGyroY());
		OD_IMU_GyroZ_set(imuMaster.getGyroZ());

}

}

