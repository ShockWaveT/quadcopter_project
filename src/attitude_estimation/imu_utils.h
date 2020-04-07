/*
 * dsp_utils.h
 *
 *  Created on: 05-Apr-2020
 *      Author: ASUS
 */

#ifndef ATTITUDE_ESTIMATION_IMU_UTILS_H_
#define ATTITUDE_ESTIMATION_IMU_UTILS_H_

void low_pass_filter(float CurrentData, float* filteredData, float filterCoeff);
void gyro_caliberate(int16_t* gyroData, float* calibValue);
void accel_caliberate(int16_t* accelData, float* calibValue);
float convert_degrees_to_radians(float degrees);
float convert_radians_to_degrees(float radianVal);

#endif /* ATTITUDE_ESTIMATION_IMU_UTILS_H_ */
