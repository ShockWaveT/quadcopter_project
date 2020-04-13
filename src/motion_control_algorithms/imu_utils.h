/*
 * dsp_utils.h
 *
 *  Created on: 05-Apr-2020
 *      Author: ASUS
 */

#ifndef MOTION_CONTROL_ALGORITHMS_IMU_UTILS_H_
#define MOTION_CONTROL_ALGORITHMS_IMU_UTILS_H_

#include <stdint.h>

typedef struct
{
	float kp;
	float ki;
	float kd;
	float maxOutput;
	float minOutput;

	float integralErr;
	float previousErr;
	uint32_t previousTime;
	uint32_t deltaTime;
}pidStruct;

void low_pass_filter(float CurrentData, float* filteredData, float filterCoeff);
float convert_degrees_to_radians(float degrees);
float convert_radians_to_degrees(float radianVal);
float pid_compute(float input, float setPoint);


#endif /* MOTION_CONTROL_ALGORITHMS_IMU_UTILS_H_ */
