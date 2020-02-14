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

/*
 * @param sensitivity is gyro full scale range.
 * 0x00: 250 degrees/second, error: 130
 * 0x08: 500 degrees/second, error: NA
 * 0x10: 1000 degrees/second, error: 26
 * 0x18: 2000 degrees/second, error: 13
 */
int8_t gyro_init(uint8_t fullScaleRange)
{
	int8_t returnCode=0;

	/* turn off sleep mode */
	returnCode = i2c_slave_mem_write(MPU6050_ID, 0x6B, 0);
	if(returnCode == -1)
		return -1;

	returnCode = i2c_slave_mem_write(MPU6050_ID, 0x1B, fullScaleRange);
	if(returnCode == -1)
		return -1;
	uart_printf("2\n");

	return 0;
}

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



