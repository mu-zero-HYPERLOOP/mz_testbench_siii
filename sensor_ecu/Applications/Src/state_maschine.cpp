/*
 * state_maschine.cpp
 *
 *  Created on: May 11, 2023
 *      Author: OfficeLaptop
 */

#include "state_maschine.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "sdc.hpp"
#include "brake_ecu_remote.hpp"
#include "cooling_controll.hpp"
#include "kistler_remote.hpp"
#include "clu_remote.hpp"
#include "ground_station_remote.hpp"
#include "canzero.hpp"
#include "pdu_remote.hpp"



namespace state_maschine {

static constexpr TickType_t STARTUP_TIME = pdMS_TO_TICKS(1000);
static constexpr float MIN_SPEED_TO_TRANSITION_INTO_CRUSING = 3;
static constexpr float DISTANCE_TO_STOP_LEVITATION = 4.0;
static constexpr float STRIPE_COUNT_TO_STOP_LEVITATION = 4;
static constexpr TickType_t TIME_TO_DISENGAGE_BRAKES = pdMS_TO_TICKS(1000);
static constexpr TickType_t TIME_TO_ENGAGE_BRAKES = pdMS_TO_TICKS(1000);
static constexpr float SPEED_TO_ENGAGE_BRAKES = 10;
static constexpr TickType_t MAX_TIME_ROLLING = pdMS_TO_TICKS(2000);
static constexpr TickType_t DISTANCE_TO_FORCE_BRAKES = 10;
static constexpr TickType_t STRIPE_COUNT_TO_FORCE_BRAKES = 10;
static constexpr float SPEED_CONSIDERED_STOPED = 0.1;


static PodState s_state;
static PodState s_lastState;
static PodState s_nextState;
static volatile bool s_error = false;
static osMutexId_t s_stateMutex = osMutexNew(NULL);

static TickType_t s_lastTransition = 0;

void setState(PodState state){
    osMutexAcquire(s_stateMutex, portMAX_DELAY);
	s_nextState = state;
    osMutexRelease(s_stateMutex);
    ground::reset();
}

void update(){
    osMutexAcquire(s_stateMutex, portMAX_DELAY);
	if(s_state != s_nextState){
		s_lastTransition = xTaskGetTickCount();
	}
	s_lastState = s_state;
	s_state = s_nextState;
    osMutexRelease(s_stateMutex);
    bool handlingError = s_error;

	TickType_t timeSinceLastTransition = xTaskGetTickCount() - s_lastTransition;

    switch(s_state){
    case STATE::POD_STARTUP:
    	// TODO implement special handling of iwdg resets.
    	if(timeSinceLastTransition > STARTUP_TIME){
    		setState(STATE::POD_IDLE);
    	}
    	break;
    case STATE::POD_IDLE:
    	if(handlingError){
    		setState(STATE::POD_OFF);
    		break;
    	}
    	sdc::open();
    	brake::engageBrakes();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::disable();
    	clu::requestAction(clu::Action::MDB_STOP);
    	if(ground::lastCommand() == ground::Command::COMMAND_PREPARE){
    		setState(STATE::POD_LAUNCH_PREPARATION);
    	}
    	break;
    case STATE::POD_LAUNCH_PREPARATION:
    	if(handlingError){
    		setState(STATE::POD_IDLE);
    		break;
    	}
    	sdc::close();
    	brake::engageBrakes();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::enable();
    	clu::requestAction(clu::Action::MDB_STOP);
    	//TODO add check for track.
    	//TODO add check for temperatures.
    	if(clu::getState() == clu::State::MDB_READY){
    		setState(STATE::POD_READY_TO_LAUNCH);
    	}
    	break;
    case STATE::POD_READY_TO_LAUNCH:
    	if(handlingError){
    		setState(STATE::POD_IDLE);
    		break;
    	}
    	sdc::close();
    	brake::engageBrakes();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::enable();
    	clu::requestAction(clu::Action::MDB_STOP);
    	if(ground::lastCommand() == ground::Command::COMMAND_LAUNCH){
    		setState(STATE::POD_START_LEVITATION);
    	}
    	break;
    case STATE::POD_START_LEVITATION:
    	if(handlingError){
    		setState(STATE::POD_ROLLING);
    		break;
    	}
    	sdc::close();
    	brake::engageBrakes();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	clu::requestAction(clu::Action::MDB_START);
    	if(clu::getState() == clu::State::MDB_STABLE_LEV){
    		setState(STATE::POD_STABLE_LEVITATION);
    	}
    	break;
    case STATE::POD_STABLE_LEVITATION:
    	if(handlingError){
    		setState(STATE::POD_ROLLING);
    		break;
    	}
    	sdc::close();
    	brake::engageBrakes();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	clu::requestAction(clu::Action::MDB_START);
    	if(OD_Velocity_get() > MIN_SPEED_TO_TRANSITION_INTO_CRUSING){
    		setState(STATE::POD_CRUSING);
    	}
    	break;
    case STATE::POD_CRUSING:
    	if(handlingError){
    		setState(STATE::POD_ROLLING);
    		break;
    	}
    	sdc::close();
    	brake::engageBrakes();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	clu::requestAction(clu::Action::MDB_START);
    	if(OD_Position_get() >= DISTANCE_TO_STOP_LEVITATION || OD_StripeCount_get() >= STRIPE_COUNT_TO_STOP_LEVITATION){
    		setState(STATE::POD_DISENGAGE_BRAKES);
    	}
    	break;
    case STATE::POD_DISENGAGE_BRAKES:
    	if(handlingError){
    		setState(STATE::POD_ROLLING);
    		break;
    	}
    	sdc::close();
    	brake::disengageBrakes();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	clu::requestAction(clu::Action::MDB_START);
    	if(timeSinceLastTransition > TIME_TO_DISENGAGE_BRAKES){
    		setState(STATE::POD_STOP_LEVITATION);
    	}
    	break;
    case STATE::POD_STOP_LEVITATION:
    	if(handlingError){
    		setState(STATE::POD_ROLLING);
    		break;
    	}
    	sdc::close();
    	brake::disengageBrakes();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	clu::requestAction(clu::Action::MDB_STOP);
    	if(clu::getState() == clu::State::MDB_GROUNDED){
    		setState(STATE::POD_ROLLING);
    	}
    	break;
    case STATE::POD_ROLLING:
    	if(handlingError){
    		// normal behavior already leads to the fastest possible stop.
    	}
    	sdc::open();
    	brake::disengageBrakes();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	clu::requestAction(clu::Action::MDB_STOP);
    	if(OD_Velocity_get() <= SPEED_TO_ENGAGE_BRAKES || OD_Position_get() >= DISTANCE_TO_FORCE_BRAKES
    			|| timeSinceLastTransition > MAX_TIME_ROLLING
				|| OD_StripeCount_get() >= STRIPE_COUNT_TO_FORCE_BRAKES){
    		setState(STATE::POD_ENGAGE_BRAKES);
    	}
    	break;
    case STATE::POD_ENGAGE_BRAKES:
    	if(handlingError){
    		// normal behavior already leads to the fastest possible stop.
    	}
    	sdc::open();
    	brake::engageBrakes();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	clu::requestAction(clu::Action::MDB_STOP);
    	if(OD_Velocity_get() <= SPEED_CONSIDERED_STOPED){
    		setState(STATE::POD_END_OF_RUN);
    	}
    	break;
    case STATE::POD_END_OF_RUN:
    	if(handlingError){
    		setState(STATE::POD_OFF);
    		break;
    	}
    	sdc::open();
    	brake::engageBrakes();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::disable();
    	clu::requestAction(clu::Action::MDB_STOP);
    	break;
    case STATE::POD_SAFE_TO_APPROCH:
    	if(handlingError){
    		setState(STATE::POD_OFF);
    		break;
    	}
    	sdc::open();
    	brake::engageBrakes();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::disable();
    	clu::requestAction(clu::Action::MDB_STOP);
    	break;
    case STATE::POD_PUSHABLE:
    	if(handlingError){
    		setState(STATE::POD_OFF);
    		break;
    	}
    	sdc::open();
    	brake::disengageBrakes();
    	cooling::setMode(cooling::MODE::OFF);
    	kistler::disable();
    	clu::requestAction(clu::Action::MDB_STOP);
    	break;
    case STATE::POD_OFF:
    	//TODO disable the lp channel that supplies power to the microcontrollers.
    	//pdu::disableChannel(pdu::LP_CHANNEL1) ;
    	//turn off all microcontrollers.
    	break;
    }
    if(handlingError){
    	s_error = false;
    }
}

void start(){
	s_state = s_nextState;
	s_lastState = s_nextState;
	while(true){
		update();
		can::Message<can::messages::SensorF_TX_StatePod> stateMsg;
		stateMsg.set<can::signals::SensorF_TX_PodState>(s_state);
		stateMsg.set<can::signals::SensorF_TX_PodState_Target>(s_nextState);
		stateMsg.set<can::signals::SensorF_TX_PodState_Last>(s_lastState);
		stateMsg.send();
	}
}

}


// overwrites weak implementation from cz_weak.cpp
void canzero::handle_emergency_warning(){
	state_maschine::s_error = true;
	ERR_ALL_clear();
}
