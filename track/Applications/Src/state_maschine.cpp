/*
 * state_maschine.hpp
 *
 *  Created on: May 19, 2023
 *      Author: OfficeLaptop
 */
#include "state_maschine.hpp"
#include "State.hpp"
#include "ground_station_remote.hpp"
#include "solenoid.hpp"

namespace state_maschine {

constexpr float MIN_PROPULSION_PRESSURE = 4;
constexpr float MIN_PROPULSION_DISTANCE = 0.00;
constexpr float MAX_PROPULSION_DISTANCE = 0.03;

void init() {

}

void update() {
	PodState state = ground_station::getState();

	// extend piston.
	// TODO implement low level control of the track.
	switch (state) {
	case STATE::POD_OFF:
	case STATE::POD_IDLE:
	case STATE::POD_LAUNCH_PREPARATION:
	case STATE::POD_FAULT:
	case STATE::POD_READY_TO_LAUNCH:
	case STATE::POD_LAUNCHING:
	{
		solenoid::PistonStatus pistonStatus =
				static_cast<solenoid::PistonStatus>(OD_PistonStatus_get());
		if(pistonStatus == solenoid::PistonStatus::Ready){
			float distance = OD_PropulsionDistance_get();
			if(distance >= MIN_PROPULSION_DISTANCE && distance <= MAX_PROPULSION_DISTANCE){
				float pressure = OD_PropulsionPressure_get();
				if(pressure >= MIN_PROPULSION_DISTANCE){
					bool allowed = solenoid::push();
					if(!allowed){
						// set error flag
						ERR_OtherError_set();
					}
				}
			}
		}
		break;
	}
	case STATE::POD_PUSHABLE:
	case STATE::POD_SAFE_TO_APPROACH:
	case STATE::POD_START_LEVITATION:
	case STATE::POD_STOP_LEVITATION:
	case STATE::POD_LEVITATING:
	case STATE::POD_BREAKING:
	case STATE::POD_STOP:
	default:
		break;
	}
}

}
