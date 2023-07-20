/*
 * imu_controll.hpp
 *
 *  Created on: May 18, 2023
 *      Author: OfficeLaptop
 */

#include <imu.hpp>
#include "ImuMaster.hpp"
#include "canzero.hpp"
#include "estdio.hpp"

namespace imu {

static constexpr bool FREQUENT_LOGGING = true;

ImuMaster imuMaster;

void init(){
	imuMaster.start();
	can::Message<can::messages::SensorF_SDO_Resp> numberMsg;
	numberMsg.set<can::signals::SensorF_OD_IMU_number>(imuMaster.onlineCount());
	numberMsg.send();
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

		if(FREQUENT_LOGGING){
			can::Message<can::messages::SensorF_SDO_Resp> axMsg;
			axMsg.set<can::signals::SensorF_SDO_ID>(can::signals::SensorF_SDO_ID::IMU_ACCELX);
			axMsg.set<can::signals::SensorF_OD_IMU_AccelX>(OD_IMU_AccelX_get());
			axMsg.send();
			can::Message<can::messages::SensorF_SDO_Resp> ayMsg;
			ayMsg.set<can::signals::SensorF_SDO_ID>(can::signals::SensorF_SDO_ID::IMU_ACCELY);
			ayMsg.set<can::signals::SensorF_OD_IMU_AccelY>(OD_IMU_AccelY_get());
			ayMsg.send();

			can::Message<can::messages::SensorF_SDO_Resp> azMsg;
			azMsg.set<can::signals::SensorF_SDO_ID>(can::signals::SensorF_SDO_ID::IMU_ACCELZ);
			azMsg.set<can::signals::SensorF_OD_IMU_AccelZ>(OD_IMU_AccelZ_get());
			azMsg.send();

			can::Message<can::messages::SensorF_SDO_Resp> gxMsg;
			gxMsg.set<can::signals::SensorF_SDO_ID>(can::signals::SensorF_SDO_ID::IMU_GYROX);
			gxMsg.set<can::signals::SensorF_OD_IMU_GyroX>(OD_IMU_GyroX_get());
			gxMsg.send();

			can::Message<can::messages::SensorF_SDO_Resp> gyMsg;
			gyMsg.set<can::signals::SensorF_SDO_ID>(can::signals::SensorF_SDO_ID::IMU_GYROY);
			gyMsg.set<can::signals::SensorF_OD_IMU_GyroY>(OD_IMU_GyroY_get());
			gyMsg.send();

			can::Message<can::messages::SensorF_SDO_Resp> gzMsg;
			gzMsg.set<can::signals::SensorF_SDO_ID>(can::signals::SensorF_SDO_ID::IMU_GYROZ);
			gzMsg.set<can::signals::SensorF_OD_IMU_GyroZ>(OD_IMU_GyroZ_get());
			gzMsg.send();
		}
}

}

