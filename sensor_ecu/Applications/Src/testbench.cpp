/*
 * testbench.cpp
 *
 *  Created on: Apr 17, 2023
 *      Author: karl
 */

#include <OnBoardSensors.hpp>
#include <peripheral_config.hpp>
#include "StateMaschine.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include <cinttypes>
#include "canzero.hpp"
#include "FiducialSensor.hpp"
#include "estdio.hpp"
#include "StartupState.hpp"
#include "AdcModuleController.hpp"
#include "AdcChannelController.hpp"
#include "GPIOWriteController.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void testbench_entry(void *argv) {
	OnBoardSensors onBoardSensors(g_peripherals.m_onBoardTemperaturConfig);

	while (true) {
		float internal = onBoardSensors.getInternalTemperaturC();
		float external = onBoardSensors.getExternalTemperaturC();
		float voltage = onBoardSensors.getInputVoltage();

		onBoardSensors.updateODs();

		printf("%f  %f  %f \n", internal, external, voltage);
		osDelay(1000);
	}
}

#ifdef __cplusplus
}
#endif
