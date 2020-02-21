/*
 * gy87_module.c
 *
 *  Created on: 05-Jan-2020
 *      Author: arun
 */

#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "gy87_module.h"
#include "timer_config.h"
#include "i2c_comm.h"
#include "small_printf.h"

/**
 * @param gyroFullScaleRange: gyro full scale range.
 * @param accelFullScaleRange: accelerometer full scale range.
 * @retval 0 on success, -1 on timeout.
 *
 * 0x00: 250 degrees/second, error: 130
 * 0x08: 500 degrees/second, error: NA
 * 0x10: 1000 degrees/second, error: 26
 * 0x18: 2000 degrees/second, error: 13
 *
 * accelerometer
 * 0x00:  error:
 * 0x08:  error: x-280, y-60, z-7380
 * 0x10:  error:
 * 0x18:  error:
 *
 */
int8_t mpu6050_init(uint8_t gyroFullScaleRange, uint8_t accelFullScaleRange)
{
	int8_t returnCode=0;

	uart_printf("f1\n");

	/* turn off sleep mode */
	returnCode = i2c_slave_mem_write(MPU6050_ID, 0x6B, 0);
	if(returnCode == -1)
		return -1;

	uart_printf("f2\n");

	returnCode = i2c_slave_mem_write(MPU6050_ID, 0x1B, gyroFullScaleRange);
	if(returnCode == -1)
		return -1;



	returnCode = i2c_slave_mem_write(MPU6050_ID, 0x1C, accelFullScaleRange);
	if(returnCode == -1)
		return -1;



	return 0;
}

/**
 * Reads the raw X, Y and Z angular velocity from gyro.
 * @param gyroBuffer: buffer for the read data to be stored.
 * @retval 0 on success, -1 on timeout.
 */
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

/**
 * Reads the raw X, Y and Z values from accelerometer.
 * @param accelBuffer: buffer for the read data to be stored.
 * @retval 0 on success, -1 on timeout.
 */
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



