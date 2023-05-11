/*
 * TimPeriodElapsedCallbackHandler.cpp
 *
 *  Created on: Apr 18, 2023
 *      Author: OfficeLaptop
 */


#include "TimPeriodElapsedCallbackHandler.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	TimPeriodElapsedCallbackHandler::getInstance().notify(htim);
}

#ifdef __cplusplus
}
#endif
