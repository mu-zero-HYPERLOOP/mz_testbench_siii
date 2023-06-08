/*
 * state_maschine.cpp
 *
 *  Created on: May 11, 2023
 *      Author: OfficeLaptop
 */

#include "state_maschine.hpp"
#include "estdio.hpp"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "sdc.hpp"
#include "cooling_controll.hpp"
#include "kistler_remote.hpp"
#include "clu_remote.hpp"
#include "ground_station_remote.hpp"
#include "canzero.hpp"
#include "pdu_remote.hpp"



namespace state_maschine {

static constexpr TickType_t STATE_BRD_INTERVAL = pdMS_TO_TICKS(100);

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
static volatile bool s_handlingError = false;
static osMutexId_t s_stateMutex = osMutexNew(NULL);

static TickType_t s_lastTransition = 0;
static TickType_t s_lastBrd = 0;

void setState(PodState state){
    osMutexAcquire(s_stateMutex, portMAX_DELAY);
    s_nextState = state;
    osMutexRelease(s_stateMutex);
    ground::reset();
}

void init(){
	s_nextState = STATE::POD_STARTUP;
}

void update(){
    osMutexAcquire(s_stateMutex, portMAX_DELAY);
	if(s_state != s_nextState){
		s_state = s_nextState;
		s_lastTransition = xTaskGetTickCount();
		s_lastBrd = 0;
	}
	s_lastState = s_state;
    osMutexRelease(s_stateMutex);
    bool handlingError = s_error;

	TickType_t timeSinceLastTransition = xTaskGetTickCount() - s_lastTransition;
	TickType_t timeSinceLastBrd = xTaskGetTickCount() - s_lastBrd;
	if(timeSinceLastBrd > STATE_BRD_INTERVAL){
		can::Message<can::messages::SensorF_TX_StatePod> msg;
		msg.set<can::signals::SensorF_TX_PodState>(s_state);
		msg.send();
		s_lastBrd = xTaskGetTickCount();
	}

    switch(s_state){
    case STATE::POD_STARTUP:
    	// TODO implement special handling of iwdg resets.
    	if(timeSinceLastTransition > STARTUP_TIME){
    		setState(STATE::POD_IDLE);
    	}
    	break;
    case STATE::POD_IDLE:
    	sdc::open();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::disable();
    	if(ground::lastCommand() == ground::COMMAND::LAUNCH_PREPARATION){
    		setState(STATE::POD_LAUNCH_PREPARATION);
    	}
    	break;
    case STATE::POD_LAUNCH_PREPARATION:
    	sdc::close();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::enable();
    	//TODO add check for track.
    	//TODO add check for temperatures.
    	if(clu::getState() == clu::State::MDB_READY){
    		setState(STATE::POD_READY_TO_LAUNCH);
    	}
    	break;
    case STATE::POD_READY_TO_LAUNCH:
    	sdc::close();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::enable();
    	if(ground::lastCommand() == ground::COMMAND::LAUNCH_START){
    		setState(STATE::POD_START_LEVITATION);
    	}
    	break;
    case STATE::POD_START_LEVITATION:
    	sdc::close();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	if(clu::getState() == clu::State::MDB_STABLE_LEV){
    		setState(STATE::POD_STABLE_LEVITATION);
    	}
    	break;
    case STATE::POD_STABLE_LEVITATION:
    	sdc::close();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	if(OD_Velocity_get() > MIN_SPEED_TO_TRANSITION_INTO_CRUSING){
    		setState(STATE::POD_CRUSING);
    	}
    	break;
    case STATE::POD_CRUSING:
    	sdc::close();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	if(OD_Position_get() >= DISTANCE_TO_STOP_LEVITATION || OD_StripeCount_get() >= STRIPE_COUNT_TO_STOP_LEVITATION){
    		setState(STATE::POD_DISENGAGE_BRAKES);
    	}
    	break;
    case STATE::POD_DISENGAGE_BRAKES:
    	sdc::close();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	if(timeSinceLastTransition > TIME_TO_DISENGAGE_BRAKES){
    		setState(STATE::POD_STOP_LEVITATION);
    	}
    	break;
    case STATE::POD_STOP_LEVITATION:
    	sdc::close();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	if(clu::getState() == clu::State::MDB_GROUNDED){
    		setState(STATE::POD_ROLLING);
    	}
    	break;
    case STATE::POD_ROLLING:
    	sdc::open();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	if(OD_Velocity_get() <= SPEED_TO_ENGAGE_BRAKES || OD_Position_get() >= DISTANCE_TO_FORCE_BRAKES
    			|| timeSinceLastTransition > MAX_TIME_ROLLING
				|| OD_StripeCount_get() >= STRIPE_COUNT_TO_FORCE_BRAKES){
    		setState(STATE::POD_ENGAGE_BRAKES);
    	}
    	break;
    case STATE::POD_ENGAGE_BRAKES:
    	sdc::open();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::enable();
    	if(OD_Velocity_get() <= SPEED_CONSIDERED_STOPED){
    		setState(STATE::POD_END_OF_RUN);
    	}
    	break;
    case STATE::POD_END_OF_RUN:
    	sdc::open();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::disable();
    	break;
    case STATE::POD_SAFE_TO_APPROCH:
    	sdc::open();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::disable();
    	break;
    case STATE::POD_PUSHABLE:
    	sdc::open();
    	cooling::setMode(cooling::MODE::OFF);
    	kistler::disable();
    	break;
    case STATE::POD_OFF:
    	//pdu::killMe();
    	break;
    }
}

}
