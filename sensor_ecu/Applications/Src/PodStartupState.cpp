/*
 * PodStartupState.cpp
 *
 *  Created on: May 2, 2023
 *      Author: OfficeLaptop
 */

#include <PodStartupState.hpp>
#include "GlobalState.hpp"
#include "PodIdleState.hpp"

void PodStartupState::setup() {
	printf("enter startup state\n");
}

void PodStartupState::update() {
	GlobalState::getInstance().setState<PodIdleState>();
}

void PodStartupState::dispose() {

	printf("exit startup state\n");
}

