/*
 * @file 	imu_utils.c
 * @date 	05-April-2020
 * @author 	Arun Cheriyan
 */
#include <math.h>
#include "stm32f10x.h"
#include "imu_utils.h"


/************************************************************************//*
 * calculates new filtered data from previous filtered data and
 * current raw data.
 *
 * @param CurrentData: unfiltered data/raw data
 * @param PreviousData: previously filtered data
 * @param filterCoeff: filter coefficient(the lower this becomes the slower
 * the filter)
 *
 * @retval new filtered data.
 **************************************************************************/
float low_pass_filter(float CurrentData, float PreviousData, float filterCoeff)
{
	return (filterCoeff*CurrentData)+((1-filterCoeff)*PreviousData);
}


/************************************************************************//*
 * removes bias error from raw gyro data. bias must be calculated
 * previously
 *
 * @param GyroData: raw gyro data array
 * @param calibValue: array of previously calculated gyro bias
 *
 * @retval none.
 **************************************************************************/
void gyro_caliberate(int16_t* gyroData, float* calibValue)
{
	gyroData[0] = (gyroData[0])-calibValue[0];
	gyroData[1] = (gyroData[1])-calibValue[1];
	gyroData[2] = (gyroData[2])-calibValue[2];
}


/************************************************************************//*
 * removes bias error from raw accelerometer data. bias must be calculated
 * previously
 *
 * @param accelData: raw accelerometer data array
 * @param calibValue: array of previously calculated acceleromter bias
 *
 * @retval none.
 **************************************************************************/
void accel_caliberate(int16_t* accelData, float* calibValue)
{
	accelData[0] = accelData[0]+calibValue[0];
	accelData[1] = accelData[1]+calibValue[1];
	accelData[2] = accelData[2]+calibValue[2];
}


/************************************************************************//*
 * can be used to convert degrees to radians or dgrees/second to
 * radians/second
 *
 * @param degrees: value to be converted in degrees
 *
 * @retval value in radians.
 **************************************************************************/
float convert_degrees_to_radians(uint32_t degrees)
{
	return (((float)degrees)*M_PI)/180;
}

