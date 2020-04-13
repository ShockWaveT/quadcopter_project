/**
 * @file 	imu_utils.c
 * @date 	05-April-2020
 * @author 	Arun Cheriyan
 */
#include <math.h>
#include "stm32f10x.h"
#include "motion_control_algorithms/imu_utils.h"
#include "timer_config.h"


/************************************************************************//*
 * calculates new filtered data from previous filtered data and
 * current raw data.
 *
 * @param CurrentData: unfiltered data/raw data
 * @param filteredData: previously filtered data. new filtered data
 * is stored here
 * @param filterCoeff: filter coefficient(the lower this becomes the slower
 * the filter)
 *
 * @retval none.
 **************************************************************************/
void low_pass_filter(float CurrentData, float* filteredData, float filterCoeff)
{
	(*filteredData) = (filterCoeff*CurrentData)+((1-filterCoeff)*(*filteredData));
}


/************************************************************************//*
 * can be used to convert degrees to radians or dgrees/second to
 * radians/second
 *
 * @param degrees: value to be converted in degrees
 *
 * @retval value in radians.
 **************************************************************************/
float convert_degrees_to_radians(float degrees)
{
	/* radians = (degrees*PI)/180 */
	//return (degrees*M_PI)/180;
	return (degrees*0.01745);
}


/************************************************************************//*
 * can be used to convert radians to degrees or radians/second to
 * degrees/second
 *
 * @param radianVal: value to be converted in radians
 *
 * @retval value in degrees.
 **************************************************************************/
float convert_radians_to_degrees(float radianVal)
{
	/* degrees = (radians*180)/PI */
	/* return (radianVal*180)/M_PI */;
	return (radianVal*57.29577);

}


float pid_compute(float input, float setPoint)
{
	float currentErr;
	static float previousErr;

	uint32_t currentTime;
	static uint32_t previousTime;
	float timeDifference;

	const float kp=10, ki=0.1, kd=0.1;
	const float maxOutput = 180;
	const float minOutput = 0;

	float output;
	static float integralErr=0;
	float differentialErr=0;

	currentTime = millis();
	timeDifference = currentTime-previousTime;
	previousTime = currentTime;

	currentErr = setPoint-input;

	integralErr = integralErr+currentErr;
	differentialErr = currentErr-previousErr;
	previousErr = currentErr;

	if(timeDifference>0)
	{
		output = (kp*currentErr) + (ki*integralErr*timeDifference) + ((kd*differentialErr)/timeDifference);

		if(output>maxOutput)
			output = maxOutput;

		else if(output<minOutput)
			output = minOutput;
	}

	return output;
}
