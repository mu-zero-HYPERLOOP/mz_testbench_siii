/*
 * IMU.cpp
 *
 *  Created on: 20 Apr 2022
 *      Author: piril
 */

#include "IMU.hpp"
#include "spi.h"
#include "log.h"


//TODO: check how big the buffer should be
#define SPI_MSG_LENGTH 8
IMU::IMU(){
}


void IMU::receiveData(void* params){
	const uint8_t imuNr = *((uint8_t*) params);
	extern osMutexId_t IMUMutex;
	uint8_t rxDataBuf[SPI_MSG_LENGTH];
	uint8_t txDataBuf[SPI_MSG_LENGTH];
	GPIO_TypeDef* csIMUPort;
	uint16_t csIMUPin;
	switch(imuNr){
	case 1:
		HAL_GPIO_WritePin(CS_IMU1_GPIO_Port, CS_IMU1_Pin, GPIO_PIN_SET);
		csIMUPort = CS_IMU1_GPIO_Port;
		csIMUPin = CS_IMU1_Pin;
		break;
	case 2:
		HAL_GPIO_WritePin(CS_IMU2_GPIO_Port, CS_IMU2_Pin, GPIO_PIN_SET);
		csIMUPort = CS_IMU2_GPIO_Port;
		csIMUPin = CS_IMU2_Pin;
		break;
	case 3:
		HAL_GPIO_WritePin(CS_IMU3_GPIO_Port, CS_IMU3_Pin, GPIO_PIN_SET);
		csIMUPort = CS_IMU3_GPIO_Port;
		csIMUPin = CS_IMU3_Pin;
		break;
	default:
		printDebug("only IMU 1-3 are available!\n");
		osThreadExit();
		return;
	}
	while(1){
		//TODO: set txBuffer to the imu request
		osMutexAcquire(IMUMutex, portMAX_DELAY);
		HAL_GPIO_WritePin(csIMUPort, csIMUPin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi2, txDataBuf, rxDataBuf, SPI_MSG_LENGTH, portMAX_DELAY);
		//HAL_SPI_Receive(hspi2, dataBuf, SPI_MSG_LENGTH, 10);
		HAL_GPIO_WritePin(csIMUPort, csIMUPin, GPIO_PIN_SET);
		osMutexRelease(IMUMutex);
	}
}

void IMU::updatePosition(void* params){

}

IMU::~IMU() {
	// TODO Auto-generated destructor stub
}



