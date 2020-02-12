/*
 * gy87_module.h
 *
 *  Created on: 05-Jan-2020
 *      Author: arun
 */

#ifndef GY87_MODULE_H_
#define GY87_MODULE_H_

#define MPU6050_ID 0x68
#define BMP180_ID 0x77

int8_t gyro_init(uint8_t sensitivity);
int8_t gyro_measurement_read(int16_t* gyroBuffer);



#endif /* GY87_MODULE_H_ */
