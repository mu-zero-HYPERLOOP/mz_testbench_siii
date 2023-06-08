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

volatile bool pushed = false;

void init() {

}

void update() {
	PodState state = ground_station::getState();
	if(state == can::signals::SensorF_TX_PodState::POD_STABLE_LEVITATION){
		if(not pushed){
			pushed = true;
			solenoid::push();
		}
	}else {
		pushed = false;
	}
}

}
