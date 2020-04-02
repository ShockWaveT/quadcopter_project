/*
 * @file 	gy87_module.c
 * @date 	05-Jan-2020
 * @author 	Arun Cheriyan
 */

#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "gy87_module.h"
#include "timer_config.h"
#include "i2c_comm.h"
#include "small_printf.h"


/************************************************************************//*
 * initializes the gyro and accelerometer devices of the mpu6050
 * module
 *
 * @param gyroFullScaleRange: gyro full scale range.
 * @param accelFullScaleRange: accelerometer full scale range.
 * @retval 0 on success, -1 on timeout.
 *
 * 0x00: 250 degrees/second, error: 130
 * 0x08: 500 degrees/second, error: NA
 * 0x10: 1000 degrees/second, error: 26
 * 0x18: 2000 degrees/second, error: 13
 *
 **************************************************************************/
int8_t mpu6050_init(uint8_t gyroFullScaleRange, uint8_t accelFullScaleRange)
{
	int8_t returnCode=0;

	/* turn off sleep mode */
	returnCode = i2c_slave_mem_write(MPU6050_ID, 0x6B, 0);
	if(returnCode == -1)
		return -1;

	returnCode = i2c_slave_mem_write(MPU6050_ID, 0x1B, gyroFullScaleRange);
	if(returnCode == -1)
		return -1;

	returnCode = i2c_slave_mem_write(MPU6050_ID, 0x1C, accelFullScaleRange);
	if(returnCode == -1)
		return -1;

	return 0;
}


/************************************************************************//*
 * Reads the raw X, Y and Z angular velocity from gyro.
 *
 * @param gyroBuffer: buffer for the read data to be stored.
 * @retval 0 on success, -1 on timeout.
 **************************************************************************/
int8_t gyro_measurement_read(int16_t* gyroBuffer)
{
	uint8_t gyroReadValues[6];
	int8_t errorValue;

	errorValue = i2c_slave_mem_read(MPU6050_ID, 0x43, gyroReadValues, 6);

	if(errorValue == 0)
	{
		gyroBuffer[0] = (gyroReadValues[0]<<8)|gyroReadValues[1];//x_out
		gyroBuffer[1] = (gyroReadValues[2]<<8)|gyroReadValues[3];//y_out
		gyroBuffer[2] = (gyroReadValues[4]<<8)|gyroReadValues[5];//z_out
		return 0;
	}
	else
		return -1;
}


/************************************************************************//*
 * Reads the raw X, Y and Z values from accelerometer.
 *
 * @param accelBuffer: buffer for the read data to be stored.
 * @retval 0 on success, -1 on timeout.
 **************************************************************************/
int8_t accel_measurement_read(int16_t* accelBuffer)
{
	uint8_t accelReadValues[6];
	int8_t errorValue;

	errorValue = i2c_slave_mem_read(MPU6050_ID, 0x3B, accelReadValues, 6);

	if(errorValue == 0)
	{
		accelBuffer[0] = (accelReadValues[0]<<8)|accelReadValues[1];//x_out
		accelBuffer[1] = (accelReadValues[2]<<8)|accelReadValues[3];//y_out
		accelBuffer[2] = (accelReadValues[4]<<8)|accelReadValues[5];//z_out
		return 0;
	}
	else
		return -1;
}


/************************************************************************//*
 * calculates gyro calibration values to compensate sensor error.
 *
 * @param gyroCalibData: gyro axis calibration values return buffer.
 * @retval 0 if success, -1 on failure.
 *
 **************************************************************************/
int8_t gyro_do_calibration(double* gyroCalibData)
{
	uint32_t sampleCount=0;
	int16_t gyroRawData[3];

	gyroCalibData[X_AXIS_INDEX]=0;
	gyroCalibData[Y_AXIS_INDEX]=0;
	gyroCalibData[Z_AXIS_INDEX]=0;

	while(sampleCount<200)
	{
		if(gyro_measurement_read(gyroRawData)<0)
			return -1;

		gyroCalibData[X_AXIS_INDEX] = (gyroRawData[X_AXIS_INDEX]+gyroCalibData[X_AXIS_INDEX]);
		gyroCalibData[Y_AXIS_INDEX] = (gyroRawData[Y_AXIS_INDEX]+gyroCalibData[Y_AXIS_INDEX]);
		gyroCalibData[Z_AXIS_INDEX] = (gyroRawData[Z_AXIS_INDEX]+gyroCalibData[Z_AXIS_INDEX]);
		sampleCount++;
		delay_ms(15);
	}

	gyroCalibData[X_AXIS_INDEX] = gyroCalibData[X_AXIS_INDEX]/200;
	gyroCalibData[Y_AXIS_INDEX] = gyroCalibData[Y_AXIS_INDEX]/200;
	gyroCalibData[Z_AXIS_INDEX] = gyroCalibData[Z_AXIS_INDEX]/200;

	return 0;
}


/************************************************************************//*
 * calculates accelerometer calibration values to compensate sensor error.
 *
 * @param accelCalibData: accelerometer axis calibration values return
 * buffer.
 * @retval 0 if success, -1 on failure.
 *
 **************************************************************************/
int8_t accel_do_calibration(double* accelCalibData)
{
	uint32_t sampleCount=0;
	int16_t accelRawData[3];

	accelCalibData[X_AXIS_INDEX]=0;
	accelCalibData[Y_AXIS_INDEX]=0;
	accelCalibData[Z_AXIS_INDEX]=0;

	while(sampleCount<200)
	{
		if(accel_measurement_read(accelRawData)<0)
			return -1;

		accelCalibData[X_AXIS_INDEX] = (accelRawData[X_AXIS_INDEX]+accelCalibData[X_AXIS_INDEX]);
		accelCalibData[Y_AXIS_INDEX] = (accelRawData[Y_AXIS_INDEX]+accelCalibData[Y_AXIS_INDEX]);
		accelCalibData[Z_AXIS_INDEX] = (accelRawData[Z_AXIS_INDEX]+accelCalibData[Z_AXIS_INDEX]);
		sampleCount++;
		delay_ms(15);
	}

	accelCalibData[X_AXIS_INDEX] = 0-(accelCalibData[X_AXIS_INDEX]/200);
	accelCalibData[Y_AXIS_INDEX] = 0-(accelCalibData[Y_AXIS_INDEX]/200);
	accelCalibData[Z_AXIS_INDEX] = 8192-(accelCalibData[Z_AXIS_INDEX]/200);

	return 0;
}


