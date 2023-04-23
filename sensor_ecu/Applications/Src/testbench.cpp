/*
 * testbench.cpp
 *
 *  Created on: Apr 17, 2023
 *      Author: karl
 */

//fixme includes
#include <log_dep.hpp>
#include "StateMaschine.hpp"
#include "AdcDmaController.hpp"
#include "AdcDmaDoubleBufferedController.hpp"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "adc.h"
#include <cinttypes>
#include "GPIOExtiController.hpp"
#include "GPIOWriteController.hpp"
#include "GPIOReadController.hpp"
#include "canzero.hpp"

#ifdef __cplusplus
extern "C" {
#endif


class State2 : public State {
public:

	void setup() {

	}

	void update() {

	}

	void dispose() {

	}
};

class State1 : public State{
public:

	State1() : fiducialRight(DIN1_GPIO_Port, DIN1_Pin){
	}

	void onFiducialRight(){

	}

	void setup() {
		fiducialRight.setExtiCallback([&](bool v) {onFiducialRight();});
	}


	void update() {
		m_stateMaschine->setState<State2>();
	}

	void dispose() {
		fiducialRight.resetExtiCallback();
	}
private:
	GPIOExtiController fiducialRight;
};


void testbench_entry(void* argv){

	/*
	StateMaschineMemory<2> fmsMemory;
	StateMaschine fms(&fmsMemory);
	State1 state1;
	State2 state2;
	fms.registerState(state1);
	fms.registerState(state2);
	fms.start<State1>();
	*/

	GPIOExtiController fiducialRight(DIN1_GPIO_Port, DIN1_Pin);
	GPIOExtiController fiducialLeft(DIN2_GPIO_Port, DIN2_Pin);

	AdcDmaController<uint16_t> adc2(&hadc2);

	fiducialRight.setExtiCallback([&](bool v){
		//exti callback
		logln("Interrupt");
	});

	while(true){
		Future<AdcResult<uint16_t>>* future = adc2.requestAdcDma();
		AdcResult<uint16_t> result = future->get();
		float ain1 = result.asVoltageF(0);
		loglnValue("ain1", ain1);
		osDelay(500);
	}

	int state = 0;
	while(true){
		if(state == 0){

		}else if(state == 1){

		}else if(state == 2){

		}
	}
}

#ifdef __cplusplus
}
#endif
