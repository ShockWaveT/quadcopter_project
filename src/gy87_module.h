/*
 *  gy87_module.h
 *
 *  Created on: 05-Jan-2020
 *  Author: Arun
 */

#ifndef GY87_MODULE_H_
#define GY87_MODULE_H_

#define MPU6050_ID	0x68
#define BMP180_ID	0x77
#define HMC5883L_ID	0x1E

/* gyro/accelerometer full scale range defines for mpu6050_init function*/
#define FS_SEL0	0x00 /* 250 degrees per second/2g */
#define FS_SEL1	0x08 /* 500 degrees per second/4g */
#define FS_SEL2	0x10 /* 1000 degrees per second/8g */
#define FS_SEL3	0x18 /* 2000 degrees per second/16g */

/* sensitivity scale factors of gyro */
#define FS_SEL0_GYRO_SCALE	131
#define FS_SEL1_GYRO_SCALE	65.5
#define FS_SEL2_GYRO_SCALE	32.8
#define FS_SEL3_GYRO_SCALE	16.4

/* sensitivity scale factors of accelerometer */
#define FS_SEL0_ACCEL_SCALE	16384
#define FS_SEL1_ACCEL_SCALE	8192
#define FS_SEL2_ACCEL_SCALE	4096
#define FS_SEL3_ACCEL_SCALE	2048

/* defines for the accelerometer and gyro data array indexes */
#define X_AXIS_INDEX 0
#define Y_AXIS_INDEX 1
#define Z_AXIS_INDEX 2

/* defines for mpu6050_aux_i2c_bus_host_access function */
#define MPU6050_HOST_AUX_BUS_CONNECT 	1
#define MPU6050_HOST_AUX_BUS_DISCONNECT	0


int8_t mpu6050_init(uint8_t gyroFullScaleRange, uint8_t accelFullScaleRange);
int8_t mpu6050_aux_i2c_bus_host_access(uint8_t aux_i2c_bus_status);
int8_t hmc5883l_init(void);
int8_t gyro_measurement_read(int16_t* gyroBuffer);
int8_t accel_measurement_read(int16_t* accelBuffer);
int8_t magneto_measurement_read(int16_t* magnetoBuffer);
int8_t imu_raw_measurement_read(int16_t* accelBuffer, int16_t* gyroBuffer);
int8_t gyro_calc_bias(float* gyroCalibData);
int8_t accel_calc_bias(float* accelCalibData);

#endif /* GY87_MODULE_H_ */
