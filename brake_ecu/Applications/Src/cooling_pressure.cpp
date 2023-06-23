/*
 * cooling_pressure.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "cooling_pressure.hpp"
#include "AdcModule.hpp"
#include <cinttypes>
#include "canzero.hpp"
#include <cmath>
#include "AdcChannelController.hpp"

namespace cooling_pressure {

constexpr AdcModule INTAKE_PRESSURE_ADC_MODULE = ADC_MODULE2;
constexpr uint16_t INTAKE_PRESSURE_ADC_RANK = 3;
constexpr float INTAKE_PRESSURE_ZERO_OFFSET = -0.015;
constexpr float INTAKE_PRESSURE_C1 = 16 * 3.3 / (2.4 * 4095); //Line slope for pressure sensor
constexpr float INTAKE_PRESSURE_C2 = -16 * 0.6 / 2.4; //Constant for pressure sensor
constexpr float INTAKE_PRESSURE_UPPER_LIMIT = 17.9; //Upper limit for CAN message
constexpr float INTAKE_PRESSURE_LOWER_LIMIT = -1.9; //Lower limit for CAN message

constexpr float INTAKE_UNDER_PRESSURE_THRESHOLD = -0.5;
constexpr TickType_t INTAKE_UNDER_PRESSURE_TIMEOUT = pdMS_TO_TICKS(1000);
TickType_t lastIntakeUnderPressureOk;

constexpr float INTAKE_OVER_PRESSURE_THRESHOLD = 3;
constexpr TickType_t INTAKE_OVER_PRESSURE_TIMEOUT = pdMS_TO_TICKS(1000);
TickType_t lastIntakeOverPressureOk;

constexpr AdcModule OUTTAKE_PRESSURE_ADC_MODULE = ADC_MODULE2;
constexpr uint16_t OUTTAKE_PRESSURE_ADC_RANK = 3;
constexpr float OUTTAKE_PRESSURE_ZERO_OFFSET = -0.015;
constexpr float OUTTAKE_PRESSURE_C1 = 16 * 3.3 / (2.4 * 4095); //Line slope for pressure sensor
constexpr float OUTTAKE_PRESSURE_C2 = -16 * 0.6 / 2.4; //Constant for pressure sensor
constexpr float OUTTAKE_PRESSURE_UPPER_LIMIT = 17.9; //Upper limit for CAN message
constexpr float OUTTAKE_PRESSURE_LOWER_LIMIT = -1.9; //Lower limit for CAN message

constexpr float OUTTAKE_UNDER_PRESSURE_THRESHOLD = -0.5;
constexpr TickType_t OUTTAKE_UNDER_PRESSURE_TIMEOUT = pdMS_TO_TICKS(1000);
TickType_t lastOuttakeUnderPressureOk;

constexpr float OUTTAKE_OVER_PRESSURE_THRESHOLD = 3;
constexpr TickType_t OUTTAKE_OVER_PRESSURE_TIMEOUT = pdMS_TO_TICKS(1000);
TickType_t lastOuttakeOverPressureOk;

static AdcChannelController intakePressureAdc;

static AdcChannelController outtakePressureAdc;

void init() {
	intakePressureAdc = AdcChannelController(INTAKE_PRESSURE_ADC_MODULE,
			INTAKE_PRESSURE_ADC_RANK);
	outtakePressureAdc = AdcChannelController(OUTTAKE_PRESSURE_ADC_MODULE,
			OUTTAKE_PRESSURE_ADC_RANK);

	lastIntakeUnderPressureOk = xTaskGetTickCount();
	lastIntakeOverPressureOk = xTaskGetTickCount();
	lastOuttakeUnderPressureOk = xTaskGetTickCount();
	lastOuttakeOverPressureOk = xTaskGetTickCount();
}

void update() {
	// read pressure sensor.
	uint16_t avalue = intakePressureAdc.get();
	float intakePressure = INTAKE_PRESSURE_C1 * avalue + INTAKE_PRESSURE_C2
			+ INTAKE_PRESSURE_ZERO_OFFSET;
	intakePressure = std::max(INTAKE_PRESSURE_LOWER_LIMIT,
			std::min(INTAKE_PRESSURE_UPPER_LIMIT, intakePressure));
	OD_IntakePressure_set(intakePressure);

	avalue = outtakePressureAdc.get();
	float outtakePressure = OUTTAKE_PRESSURE_C1 * avalue + OUTTAKE_PRESSURE_C2
			+ OUTTAKE_PRESSURE_ZERO_OFFSET;
	outtakePressure = std::max(OUTTAKE_PRESSURE_LOWER_LIMIT,
			std::min(OUTTAKE_PRESSURE_UPPER_LIMIT, outtakePressure));
	OD_OuttakePressure_set(outtakePressure);

	//throwing errors.

	{
		if (intakePressure > INTAKE_UNDER_PRESSURE_THRESHOLD) {
			lastIntakeUnderPressureOk = xTaskGetTickCount();
		}
		TickType_t timeSinceUnderPressureOk = xTaskGetTickCount()
				- lastIntakeUnderPressureOk;
		if (timeSinceUnderPressureOk > INTAKE_UNDER_PRESSURE_TIMEOUT) {
			ERR_IntakeUnderPressure_set();
		} else {
			ERR_IntakeUnderPressure_clear();
		}
	}
	{
		if (intakePressure < INTAKE_OVER_PRESSURE_THRESHOLD) {
			lastIntakeOverPressureOk = xTaskGetTickCount();
		}
		TickType_t timeSinceOverPressureOk = xTaskGetTickCount()
				- lastIntakeOverPressureOk;
		if (timeSinceOverPressureOk > INTAKE_OVER_PRESSURE_TIMEOUT) {
			ERR_IntakeOverPressure_set();
		} else {
			ERR_IntakeOverPressure_clear();
		}
	}
	{
		if(outtakePressure > OUTTAKE_UNDER_PRESSURE_THRESHOLD){
			lastOuttakeUnderPressureOk = xTaskGetTickCount();
		}
		TickType_t timeSinceUnderPressureOk = xTaskGetTickCount() - lastOuttakeUnderPressureOk;
		if(timeSinceUnderPressureOk > OUTTAKE_UNDER_PRESSURE_TIMEOUT){
			ERR_OuttakeUnderPressure_set();
		}else{
			ERR_OuttakeUnderPressure_clear();
		}
	}
	{
		if(outtakePressure < OUTTAKE_OVER_PRESSURE_THRESHOLD){
			lastOuttakeOverPressureOk = xTaskGetTickCount();
		}
		TickType_t timeSinceOverPressureOk = xTaskGetTickCount() - lastOuttakeOverPressureOk;
		if(timeSinceOverPressureOk > OUTTAKE_OVER_PRESSURE_TIMEOUT){
			ERR_OuttakeOverPressure_set();
		}else{
			ERR_OuttakeOverPressure_clear();
		}
	}

}

}
