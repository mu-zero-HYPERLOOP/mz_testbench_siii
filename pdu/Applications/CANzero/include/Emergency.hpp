/**
 * @file Emergency.h
 *
 * @date 08.12.2020
 * @author: Felix
 * @brief header of the emergency class
 */

#ifndef CANZERO_EMERGENCY_H_
#define CANZERO_EMERGENCY_H_

#include <stdint.h>
/**
 * @class Emergency
 * @brief
 *
 */
class Emergency {
public:
	Emergency();
	virtual ~Emergency();
	static Emergency* getEmergencyInstance();
	static void waitForEmergency(void* params);
	static void exampleEmergencyTask(void* params);
	static void handleEmergency(uint32_t emergencyBuffer, uint32_t lastEmergencyBuffer);

private:
	static Emergency* em;
};

#endif /* CANZERO_EMERGENCY_H_ */
