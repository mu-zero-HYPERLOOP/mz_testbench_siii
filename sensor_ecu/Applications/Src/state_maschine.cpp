/*
 * state_maschine.cpp
 *
 *  Created on: May 11, 2023
 *      Author: OfficeLaptop
 */

#include "heartbeat_monitor.hpp"
#include "error.hpp"
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
#include "brake_control.hpp"
#include "fiducials.hpp"
#include "canzero.hpp"



namespace state_maschine {

static constexpr TickType_t STATE_BRD_INTERVAL = pdMS_TO_TICKS(100);

static constexpr TickType_t STARTUP_TIME = pdMS_TO_TICKS(1000);
static constexpr float MIN_SPEED_TO_TRANSITION_INTO_CRUSING = 3;
static constexpr float DISTANCE_TO_STOP_LEVITATION = 1;
static constexpr float STRIPE_COUNT_TO_STOP_LEVITATION = 1;
static constexpr TickType_t TIME_TO_DISENGAGE_BRAKES = pdMS_TO_TICKS(500);
static constexpr TickType_t TIME_TO_ENGAGE_BRAKES = pdMS_TO_TICKS(5000);
static constexpr float SPEED_TO_ENGAGE_BRAKES = 1;
static constexpr TickType_t MAX_TIME_ROLLING = pdMS_TO_TICKS(1000);
static constexpr TickType_t DISTANCE_TO_FORCE_BRAKES = 1.5;
static constexpr TickType_t STRIPE_COUNT_TO_FORCE_BRAKES = 2;
static constexpr float SPEED_CONSIDERED_STOPED = 0.1;
static constexpr TickType_t STOP_LEVI_TIMEOUT = pdMS_TO_TICKS(5000);
static constexpr TickType_t MIN_PRECHARGE_TIME = pdMS_TO_TICKS(1000);
static constexpr TickType_t MAX_TIME_IN_EOR = pdMS_TO_TICKS(1000 * 60 * 2);

static constexpr float SPEED_THAT_REQUIRES_BRAKES = 0.25;


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

PodState getState(){
	return s_state;
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

	TickType_t timeSinceLastTransition = xTaskGetTickCount() - s_lastTransition;
	TickType_t timeSinceLastBrd = xTaskGetTickCount() - s_lastBrd;
	if(timeSinceLastBrd > STATE_BRD_INTERVAL){
		can::Message<can::messages::SensorF_TX_StatePod> msg;
		msg.set<can::signals::SensorF_TX_PodState>(s_state);
		msg.send();
		s_lastBrd = xTaskGetTickCount();
	}
	if(errors::hasEmergency() || ground::lastCommand() == ground::COMMAND::EMERGENCY){
		sdc::open();
		kistler::disable();
		brake_control::engage();
		setState(STATE::POD_OFF);
		return;
	}

    switch(s_state){
    case STATE::POD_STARTUP:
    	if(errors::hasError()){
    		setState(STATE::POD_OFF);
    		break;
    	}
    	kistler::disable();
    	brake_control::engage();
    	cooling::setMode(cooling::MODE::OFF);
    	sdc::open();
    	if(timeSinceLastTransition > STARTUP_TIME
    			&& heartbeat::telemetryConnected()){
    		setState(STATE::POD_IDLE);
    	}
    	break;
    case STATE::POD_IDLE:
    {
    	if(errors::hasError()){
    		setState(STATE::POD_OFF);
    		break;
    	}
    	if(ground::lastCommand() == ground::COMMAND::LAUNCH_PREPARATION){
    		setState(STATE::POD_LAUNCH_PREPARATION);
    		break;
    	}
    	if(ground::lastCommand() == ground::COMMAND::PUSHING_START){
    		setState(STATE::POD_PUSHABLE);
    		break;
    	}
    	fiducials::reset();
    	sdc::open();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::disable();
    	brake_control::engage();
    	break;
    }
    case STATE::POD_LAUNCH_PREPARATION:
    {
    	if(errors::hasError()){
    		setState(STATE::POD_OFF);
    		break;
    	}
    	if(ground::lastCommand() == ground::COMMAND::LAUNCH_ABORT){
    		setState(STATE::POD_IDLE);
    		break;
    	}
    	if(ground::lastCommand() == ground::COMMAND::LAUNCH_START){
    		setState(STATE::POD_IDLE);
    		break;
    	}
    	sdc::close();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::enable();
    	brake_control::engage();
    	if(clu::getState() == clu::State::MDB_READY && timeSinceLastTransition > MIN_PRECHARGE_TIME){
    		setState(STATE::POD_READY_TO_LAUNCH);
    	}
    	break;
    }
    case STATE::POD_READY_TO_LAUNCH:
    {
    	if(errors::hasError()){
    		setState(STATE::POD_OFF);
    		break;
    	}
    	if(ground::lastCommand() == ground::COMMAND::LAUNCH_ABORT){
    		setState(STATE::POD_IDLE);
    		break;
    	}
    	sdc::close();
    	cooling::setMode(cooling::MODE::OFF);
    	kistler::enable();
    	kistler::normalize();
    	brake_control::engage();
    	if(ground::lastCommand() == ground::COMMAND::LAUNCH_START){
    		setState(STATE::POD_START_LEVITATION);
    	}
    	break;
    }
    case STATE::POD_START_LEVITATION:
    {
    	if(errors::hasError()){
    		setState(STATE::POD_DISENGAGE_BRAKES);
    	}
    	if(ground::lastCommand() == ground::COMMAND::LAUNCH_ABORT){
    		setState(STATE::POD_STOP_LEVITATION);
    		break;
    	}
    	if(errors::hasError()){
    		setState(STATE::POD_DISENGAGE_BRAKES);
    		break;
    	}
    	sdc::close();
    	cooling::setMode(cooling::MODE::OFF);
    	kistler::enable();
    	brake_control::engage();
    	if(clu::getState() == clu::State::MDB_LEVI){
    		setState(STATE::POD_STABLE_LEVITATION);
    	}
    	break;
    }
    case STATE::POD_STABLE_LEVITATION:
    {
    	if(errors::hasError()){
    		setState(STATE::POD_DISENGAGE_BRAKES);
    		break;
    	}
    	if(OD_Position_get() >= DISTANCE_TO_FORCE_BRAKES || OD_StripeCount_get() >= STRIPE_COUNT_TO_FORCE_BRAKES){
    		setState(STATE::POD_ENGAGE_BRAKES);
    		break;
    	}
    	if(OD_Position_get() >= DISTANCE_TO_STOP_LEVITATION || OD_StripeCount_get() >= STRIPE_COUNT_TO_STOP_LEVITATION
    			|| ground::lastCommand() == ground::COMMAND::LAUNCH_ABORT){
    		if(OD_Velocity_get() >= SPEED_THAT_REQUIRES_BRAKES){
    			setState(STATE::POD_DISENGAGE_BRAKES);
    		}else{
    			setState(STATE::POD_STOP_LEVITATION);
    		}
    		break;
    	}
    	sdc::close();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::enable();
    	brake_control::engage();
    	if(OD_Velocity_get() > MIN_SPEED_TO_TRANSITION_INTO_CRUSING){
    		setState(STATE::POD_CRUSING);
    		break;
    	}
    	break;
    }
    case STATE::POD_CRUSING:
    {
    	if(OD_Position_get() >= DISTANCE_TO_FORCE_BRAKES || OD_StripeCount_get() >= STRIPE_COUNT_TO_FORCE_BRAKES){
    		setState(STATE::POD_ENGAGE_BRAKES);
    		break;
    	}
    	if(errors::hasError()){
    		if(OD_Velocity_get() >= SPEED_THAT_REQUIRES_BRAKES){
    			setState(STATE::POD_DISENGAGE_BRAKES);
    		}else{
    			setState(STATE::POD_STOP_LEVITATION);
    		}
    		break;
    	}
    	if(OD_Position_get() >= DISTANCE_TO_STOP_LEVITATION || OD_StripeCount_get() >= STRIPE_COUNT_TO_STOP_LEVITATION
    			|| ground::lastCommand() == ground::COMMAND::LAUNCH_ABORT){
    		if(OD_Velocity_get() >= SPEED_THAT_REQUIRES_BRAKES){
    			setState(STATE::POD_DISENGAGE_BRAKES);
    		}else{
    			setState(STATE::POD_STOP_LEVITATION);
    		}
    		break;
    	}
    	sdc::close();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::enable();
    	brake_control::engage();
    	if(OD_Position_get() >= DISTANCE_TO_STOP_LEVITATION || OD_StripeCount_get() >= STRIPE_COUNT_TO_STOP_LEVITATION){
    		if(OD_Velocity_get() >= SPEED_THAT_REQUIRES_BRAKES){
    			setState(STATE::POD_DISENGAGE_BRAKES);
    		}else{
    			setState(STATE::POD_STOP_LEVITATION);
    		}
    	}
    	break;
    }
    case STATE::POD_DISENGAGE_BRAKES:
    {
    	if(OD_Position_get() >= DISTANCE_TO_FORCE_BRAKES || OD_StripeCount_get() >= STRIPE_COUNT_TO_FORCE_BRAKES){
    		setState(STATE::POD_ENGAGE_BRAKES);
    		break;
    	}
    	sdc::close();
    	cooling::setMode(cooling::MODE::OFF);
    	kistler::enable();
    	if(timeSinceLastTransition > TIME_TO_DISENGAGE_BRAKES){
    		setState(STATE::POD_STOP_LEVITATION);
    		break;
    	}
    	brake_control::disengage();
    	break;
    }
    case STATE::POD_STOP_LEVITATION:
    {
    	if(OD_Position_get() >= DISTANCE_TO_FORCE_BRAKES || OD_StripeCount_get() >= STRIPE_COUNT_TO_FORCE_BRAKES){
    		setState(STATE::POD_ENGAGE_BRAKES);
    		break;
    	}
    	sdc::close();
    	cooling::setMode(cooling::MODE::OFF);
    	kistler::enable();
    	if(clu::getState() == clu::State::MDB_READY || timeSinceLastTransition > STOP_LEVI_TIMEOUT){
    		setState(STATE::POD_ROLLING);
    		break;
    	}
    	brake_control::disengage();
    	break;
    }
    case STATE::POD_ROLLING:
    {
    	sdc::open();
    	cooling::setMode(cooling::MODE::OFF);
    	kistler::enable();
    	if(OD_Velocity_get() <= SPEED_TO_ENGAGE_BRAKES || OD_Position_get() >= DISTANCE_TO_FORCE_BRAKES
    			|| timeSinceLastTransition > MAX_TIME_ROLLING
				|| OD_StripeCount_get() >= STRIPE_COUNT_TO_FORCE_BRAKES){
    		setState(STATE::POD_ENGAGE_BRAKES);
    		break;
    	}
    	brake_control::disengage();
    	break;
    }
    case STATE::POD_ENGAGE_BRAKES:
    {
    	sdc::open();
    	cooling::setMode(cooling::MODE::OFF);
    	kistler::enable();
    	if(OD_Velocity_get() <= SPEED_CONSIDERED_STOPED || timeSinceLastTransition >= TIME_TO_ENGAGE_BRAKES){
    		setState(STATE::POD_END_OF_RUN);
    		break;
    	}
    	brake_control::engage();
    	break;
    }
    case STATE::POD_END_OF_RUN:
    {
    	sdc::open();
    	cooling::setMode(cooling::MODE::ON);
    	kistler::disable();
    	brake_control::engage();
    	if(not errors::hasError()){
			if(not clu::requiresCooling()){
				setState(STATE::POD_SAFE_TO_APPROCH);
				break;
			}
    	}else{
    		if(timeSinceLastTransition > MAX_TIME_IN_EOR){
    			setState(STATE::POD_OFF);
    			break;
    		}
    	}
    	if(ground::lastCommand() == ground::COMMAND::LAUNCH_ABORT){
    		setState(STATE::POD_IDLE);
    		break;
    	}
    	break;
    }
    case STATE::POD_SAFE_TO_APPROCH:
    {
    	if(ground::lastCommand() == ground::COMMAND::IDLE){
    		setState(STATE::POD_IDLE);
    		break;
    	}
    	else if(ground::lastCommand() == ground::COMMAND::PUSHING_START){
    		setState(STATE::POD_PUSHABLE);
    		break;
    	}
    	sdc::open();
    	cooling::setMode(cooling::MODE::ADAPTIV);
    	kistler::disable();
    	brake_control::engage();
    	break;
    }
    case STATE::POD_PUSHABLE:
    {
    	if(errors::hasError()){
    		setState(STATE::POD_OFF);
    		break;
    	}
    	if(ground::lastCommand() == ground::COMMAND::IDLE
    			|| ground::lastCommand() == ground::COMMAND::LAUNCH_ABORT){
    		setState(STATE::POD_IDLE);
    		break;
    	}
    	sdc::open();
    	brake_control::disengage();
    	cooling::setMode(cooling::MODE::OFF);
    	kistler::disable();
    	break;
    }
    case STATE::POD_OFF:
    {
    	sdc::open();
    	kistler::disable();
    	cooling::setMode(cooling::MODE::OFF);
    	brake_control::engage();
    	//mainly handled by the pdu.
    	break;
    }
    }
}

}
