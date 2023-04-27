// Taken from: https://github.com/jimmyberg/LowPassFilter

#include "LowPassFilter.hpp"

//#define ERROR_CHECK (false)

#if ERROR_CHECK
#include "log.h"
#endif

#define M_PI_FLOAT		3.14159265358979323846f

LowPassFilter::LowPassFilter():
	output(0),
	ePow(0){}

LowPassFilter::LowPassFilter(float iCutOffFrequency, float iDeltaTime):
	output(0),
	ePow(1-expf(-iDeltaTime * 2 * M_PI_FLOAT * iCutOffFrequency))
{
	#if ERROR_CHECK
	if (iDeltaTime <= 0){
		printDebug("Warning: A LowPassFilter instance has been configured with 0 s as delta time.\n");
		ePow = 0;
	}
	if(iCutOffFrequency <= 0){
		printDebug("Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.\n");
		ePow = 0;
	}
	#endif
}

float LowPassFilter::update(float input){
	return output += (input - output) * ePow;
}

float LowPassFilter::update(float input, float deltaTime, float cutoffFrequency){
	reconfigureFilter(deltaTime, cutoffFrequency); //Changes ePow accordingly.
	return output += (input - output) * ePow;
}

void LowPassFilter::reconfigureFilter(float deltaTime, float cutoffFrequency){
	#if ERROR_CHECK
	if (deltaTime <= 0){
		printDebug("Warning: A LowPassFilter instance has been configured with 0 s as delta time.\n");
		ePow = 0;
	}
	if(cutoffFrequency <= 0){
		printDebug("Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.\n");
		ePow = 0;
	}
	#endif
	ePow = 1-expf(-deltaTime * 2 * M_PI_FLOAT * cutoffFrequency);
}
