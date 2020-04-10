/**
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
 * connects or disconnects the mpu6050 auxiliary i2c bus to the host
 * MCU i2c bus
 *
 * @param aux_i2c_bus_status: connects/disconnect command.
 * MPU6050_HOST_AUX_BUS_CONNECT or MPU6050_HOST_AUX_BUS_DISCONNECT
 * @retval 0 on success, -1 on timeout.
 *
 **************************************************************************/
int8_t mpu6050_aux_i2c_bus_host_access(uint8_t aux_i2c_bus_status)
{
	int8_t returnCode=0;

	if(aux_i2c_bus_status == 1)
	{
		/*disable mpu6050 master*/
		returnCode = i2c_slave_mem_write(MPU6050_ID, 0x6A, 0x00);
		if(returnCode == -1)
			return -1;

		/*enable auxiliary i2c bus access*/
		returnCode = i2c_slave_mem_write(MPU6050_ID, 0x37, 0x02);
		if(returnCode == -1)
			return -1;
	}

	else if(aux_i2c_bus_status == 0)
	{
		/*disable mpu6050 master*/
		returnCode = i2c_slave_mem_write(MPU6050_ID, 0x6A, 0x20);
		if(returnCode == -1)
			return -1;

		/*enable auxiliary i2c bus access*/
		returnCode = i2c_slave_mem_write(MPU6050_ID, 0x37, 0x00);
		if(returnCode == -1)
			return -1;
	}
	return 0;
}

/************************************************************************//*
 * initializes the HMC5883L magnetometer sensor
 * @retval 0 on success, -1 on timeout.
 **************************************************************************/
int8_t hmc5883l_init(void)
{
	int8_t returnCode=0;

	returnCode = i2c_slave_mem_write(HMC5883L_ID, 0x00, 0x78);
	if(returnCode == -1)
		return -1;

	returnCode = i2c_slave_mem_write(HMC5883L_ID, 0x01, 0xA0);
	if(returnCode == -1)
		return -1;

	returnCode = i2c_slave_mem_write(HMC5883L_ID, 0x02, 0x00);
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
 * Reads the raw X, Y and Z values from HMC5883L magnetometer.
 *
 * @param magnetoBuffer: buffer for the read data to be stored.
 * @retval 0 on success, -1 on timeout.
 **************************************************************************/
int8_t magneto_measurement_read(int16_t* magnetoBuffer)
{
	uint8_t magnetoReadValues[6];
	int8_t errorValue;

	errorValue = i2c_slave_mem_read(HMC5883L_ID, 0x03, magnetoReadValues, 6);

	if(errorValue == 0)
	{
		/* in HMC5883L data registers are in the order X, Z, Y */
		magnetoBuffer[0] = (magnetoReadValues[0]<<8)|magnetoReadValues[1];//x_out
		magnetoBuffer[2] = (magnetoReadValues[2]<<8)|magnetoReadValues[3];//z_out
		magnetoBuffer[1] = (magnetoReadValues[4]<<8)|magnetoReadValues[5];//y_out
		return 0;
	}
	else
		return -1;
}


/************************************************************************//*
 * Reads the raw X, Y and Z values from accelerometer and gyro.
 *
 * @param returnBuffer: buffer for the read data to be stored.
 * @retval 0 on success, -1 on timeout.
 **************************************************************************/
int8_t imu_raw_measurement_read(int16_t* accelBuffer, int16_t* gyroBuffer)
{
	uint8_t ReadValues[14];
	int8_t errorValue;

	errorValue = i2c_slave_mem_read(MPU6050_ID, 0x3B, ReadValues, 14);

	if(errorValue == 0)
	{
		accelBuffer[0] = (ReadValues[0]<<8)|ReadValues[1];//x_out
		accelBuffer[1] = (ReadValues[2]<<8)|ReadValues[3];//y_out
		accelBuffer[2] = (ReadValues[4]<<8)|ReadValues[5];//z_out

		gyroBuffer[0] = (ReadValues[8]<<8)|ReadValues[9];//x_out
		gyroBuffer[1] = (ReadValues[10]<<8)|ReadValues[11];//y_out
		gyroBuffer[2] = (ReadValues[12]<<8)|ReadValues[13];//z_out
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
int8_t gyro_calc_bias(float* gyroCalibData)
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
int8_t accel_calc_bias(float* accelCalibData)
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


/************************************************************************//*
 * calculates magnetometer maximum and minimum readings for each axis.
 * these values can then be used to caliberate the magnetometer.
 *
 * @param magMax: pointer to the array where the highest read values for
 * each axis is to be stored.
 * @param magMin: pointer to the array where the lowest read values for
 * each axis is to be stored.
 *
 * @retval 0 if success, -1 on failure.
 **************************************************************************/
int magnetometer_read_min_and_max_values(int16_t* magMax, int16_t* magMin)
{
	int16_t madData[3];
	uint32_t maxSampleCount=400;

	magMax[X_AXIS_INDEX]=0;
	magMax[Y_AXIS_INDEX]=0;
	magMax[Z_AXIS_INDEX]=0;
	magMin[X_AXIS_INDEX]=0;
	magMin[Y_AXIS_INDEX]=0;
	magMin[Z_AXIS_INDEX]=0;

	while(maxSampleCount>0)
	{
		if(magneto_measurement_read(madData)<0)
			return -1;

		if(madData[X_AXIS_INDEX] > magMax[X_AXIS_INDEX])
			magMax[X_AXIS_INDEX] = madData[X_AXIS_INDEX];

		if(madData[Y_AXIS_INDEX] > magMax[Y_AXIS_INDEX])
			magMax[Y_AXIS_INDEX] = madData[Y_AXIS_INDEX];

		if(madData[Z_AXIS_INDEX] > magMax[Z_AXIS_INDEX])
			magMax[Z_AXIS_INDEX] = madData[Z_AXIS_INDEX];

		if(madData[X_AXIS_INDEX] < magMin[X_AXIS_INDEX])
			magMin[X_AXIS_INDEX] = madData[X_AXIS_INDEX];

		if(madData[Y_AXIS_INDEX] < magMin[Y_AXIS_INDEX])
			magMin[Y_AXIS_INDEX] = madData[Y_AXIS_INDEX];

		if(madData[Z_AXIS_INDEX] < magMin[Z_AXIS_INDEX])
			magMin[Z_AXIS_INDEX] = madData[Z_AXIS_INDEX];

		delay_ms(50);
		maxSampleCount--;
	}
	return 0;
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
 * removes bias error from raw accelerometer data. bias must be calculated
 * previously
 *
 * @param magData: raw magnetometer data array. corrected data is written
 * to this same variable.
 * @param magMax: pointer to the array where the highest read values for
 * each axis is to be stored.
 * @param magMin: pointer to the array where the lowest read values for
 * each axis is to be stored.
 *
 * @retval none.
 **************************************************************************/
void magnetometer_caliberate(int16_t* magData, int16_t* magMax, int16_t* magMin)
{
	int16_t offset[3];
	int16_t avgDelta[3];
	float scaleVal[3];
	float totalAvgDelta;

	/*
	 * calculate magnetometer offset. useful for calculating soft
	 * and hard iron distortion.
	 * refer: https://appelsiini.net/2018/calibrate-magnetometer
	 */
	offset[X_AXIS_INDEX] = (magMax[X_AXIS_INDEX] + magMin[X_AXIS_INDEX])/2;
	offset[Y_AXIS_INDEX] = (magMax[Y_AXIS_INDEX] + magMin[Y_AXIS_INDEX])/2;
	offset[Z_AXIS_INDEX] = (magMax[Z_AXIS_INDEX] + magMin[Z_AXIS_INDEX])/2;

	/* below lines are for calculating soft iron distortion*/
	avgDelta[X_AXIS_INDEX] = (magMax[X_AXIS_INDEX] - magMin[X_AXIS_INDEX])/2;
	avgDelta[Y_AXIS_INDEX] = (magMax[Y_AXIS_INDEX] - magMin[Y_AXIS_INDEX])/2;
	avgDelta[Z_AXIS_INDEX] = (magMax[Z_AXIS_INDEX] - magMin[Z_AXIS_INDEX])/2;

	totalAvgDelta = avgDelta[X_AXIS_INDEX] + avgDelta[Y_AXIS_INDEX] + avgDelta[Z_AXIS_INDEX];

	scaleVal[X_AXIS_INDEX] = totalAvgDelta / avgDelta[X_AXIS_INDEX];
	scaleVal[Y_AXIS_INDEX] = totalAvgDelta / avgDelta[Y_AXIS_INDEX];
	scaleVal[Z_AXIS_INDEX] = totalAvgDelta / avgDelta[Z_AXIS_INDEX];

	/* correct the raw magnetometer readings */
	magData[X_AXIS_INDEX] = (magData[X_AXIS_INDEX] - offset[X_AXIS_INDEX]) * scaleVal[X_AXIS_INDEX];
	magData[Y_AXIS_INDEX] = (magData[Y_AXIS_INDEX] - offset[Y_AXIS_INDEX]) * scaleVal[Y_AXIS_INDEX];
	magData[Z_AXIS_INDEX] = (magData[Z_AXIS_INDEX] - offset[Z_AXIS_INDEX]) * scaleVal[Z_AXIS_INDEX];
}





