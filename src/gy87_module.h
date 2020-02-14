/*
 *  gy87_module.h
 *
 *  Created on: 05-Jan-2020
 *  Author: Arun
 */

#ifndef GY87_MODULE_H_
#define GY87_MODULE_H_

#define MPU6050_ID 0x68
#define BMP180_ID 0x77

/* full scale range defines for gyro_init function*/
#define FS_SEL0_GYRO	0x00 /*250 degrees per second*/
#define FS_SEL1_GYRO	0x08 /*500 degrees per second*/
#define FS_SEL2_GYRO	0x10 /*1000 degrees per second*/
#define FS_SEL3_GYRO	0x18 /*2000 degrees per second*/

/* sensitivity scale factors of gyro*/
#define FS_SEL0_SCALE_GYRO	131
#define FS_SEL1_SCALE_GYRO	65.5
#define FS_SEL2_SCALE_GYRO	32.8
#define FS_SEL3_SCALE_GYRO	16.4

int8_t gyro_init(uint8_t fullScaleRange);
int8_t gyro_measurement_read(int16_t* gyroBuffer);


#endif /* GY87_MODULE_H_ */
