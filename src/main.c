/*
 * main.c
 *
 * Created on: 09-Feb-2020
 * Author: Arun
 */

#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "i2c_comm.h"
#include "small_printf.h"
#include "timer_config.h"
#include "gy87_module.h"
#include "uart_comm.h"
#include "debug.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "attitude_estimation/MadgwickAHRS.h"

int16_t gyroRawData[3];
int16_t accelRawData[3];

void euler_angle_calc(double* gyroArray)
{
	double phi = (gyroArray[X_AXIS_INDEX]*M_PI)/180;
	double theta = (gyroArray[Y_AXIS_INDEX]*M_PI)/180;
	double psi = (gyroArray[Z_AXIS_INDEX]*M_PI)/180;


	double rotation_matrix[3][3] = 	{
												{ 1,		sin(phi)*tan(theta), 	cos(phi)*tan(theta) },
												{ 0, 		cos(phi), 				-sin(phi)			},
												{ 0,		sin(phi)/cos(theta),	cos(phi)/cos(theta)	},
									};




}

void test_task(void *pvParameters)
{
	while(1)
	{
		uart_printf("testing task\n");
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}

//void motion_control_task(void *pvParameters)
//{
//	double accelAngleValues[3]={0};
//	uint8_t accelCalibFlag=0;
//	double accelCalibVal[3]={0};
//	double squareOfRawAccel_X;
//	double squareOfRawAccel_Y;
//	double squareOfRawAccel_Z;
//
//	double gyroAngleValues[3]={0};
//	uint8_t gyroCalibFlag=0;
//	double gyroCalibVal[3]={0};
//	double calc1;
//	double calc2;
//
//	uint32_t previousTime=0;
//	uint32_t currentTime=0;
//	uint32_t elapsed_time_in_seconds=0;
//
//	uint32_t i=0;
//
//	while(1)
//	{
//    	if(!gyroCalibFlag)
//    	{
//    		uart_printf("gyro calibration started\n");
//    		if(gyro_do_calibration(gyroCalibVal)<0)
//    		{
//    			uart_printf("gyro calibration fail\n");
//    			while(1);
//    		}
//    		gyroCalibFlag = 1;
////    		uart_printf("calib value: %.1f\n", gyroCalibVal[X_AXIS_INDEX]);
//    	}
//    	else if(!accelCalibFlag)
//		{
//			uart_printf("accel calibration started\n");
//			if(accel_do_calibration(accelCalibVal)<0)
//			{
//				uart_printf("accel calibration fail\n");
//				while(1);
//			}
//			accelCalibFlag = 1;
////    		uart_printf("calib value: %.1f\n", gyroCalibVal[X_AXIS_INDEX]);
//		}
//
//    	else if((gyroCalibFlag == 1)&&(accelCalibFlag == 1))
//    	{
//    		if(accel_measurement_read(accelRawData)<0)
//    			uart_printf("accel read fail\n");
//
//    		accelRawData[X_AXIS_INDEX] = accelRawData[X_AXIS_INDEX]+accelCalibVal[X_AXIS_INDEX];
//    		accelRawData[Y_AXIS_INDEX] = accelRawData[Y_AXIS_INDEX]+accelCalibVal[Y_AXIS_INDEX];
//    		accelRawData[Z_AXIS_INDEX] = accelRawData[Z_AXIS_INDEX]+accelCalibVal[Z_AXIS_INDEX];
//
//			/*
//			 * This will increase speed of execution since we don't have to calculate
//			 * the squares each data more than once in one iteration. also this increases readability.
//			 */
//			squareOfRawAccel_X = accelRawData[X_AXIS_INDEX]*accelRawData[X_AXIS_INDEX];
//			squareOfRawAccel_Y = accelRawData[Y_AXIS_INDEX]*accelRawData[Y_AXIS_INDEX];
//			squareOfRawAccel_Z = accelRawData[Z_AXIS_INDEX]*accelRawData[Z_AXIS_INDEX];
//
//			accelAngleValues[Y_AXIS_INDEX] = atan(accelRawData[X_AXIS_INDEX] / sqrt((squareOfRawAccel_Y+squareOfRawAccel_Z)) );
//			accelAngleValues[X_AXIS_INDEX] = atan(accelRawData[Y_AXIS_INDEX] / sqrt((squareOfRawAccel_X+squareOfRawAccel_Z)) );
//
//			/* convert radians to degrees */
//			accelAngleValues[X_AXIS_INDEX] = accelAngleValues[X_AXIS_INDEX]*DEGREE_CNVRT_CONST;
//			accelAngleValues[Y_AXIS_INDEX] = accelAngleValues[Y_AXIS_INDEX]*DEGREE_CNVRT_CONST;
//
//			/*
//			 * delay for sampling gyro values and to give a
//			 * delay b/w reads of gyro and accelerometer.
//			 */
//			vTaskDelay(5/portTICK_PERIOD_MS);
//
//			if(gyro_measurement_read(gyroRawData)<0)
//				uart_printf("gyro read fail\n");
//
//    		currentTime = millis();
//    		elapsed_time_in_seconds = (1000/(currentTime-previousTime));
//
//    		if(elapsed_time_in_seconds!=0)
//    		{
//    			calc1 = ((double)elapsed_time_in_seconds)*16.4f;
//
//				calc2 = ((double)gyroRawData[X_AXIS_INDEX])-gyroCalibVal[X_AXIS_INDEX];
//				gyroAngleValues[X_AXIS_INDEX] = (calc2/calc1)+gyroAngleValues[X_AXIS_INDEX];
//
//				calc2 = (double)gyroRawData[Y_AXIS_INDEX]-gyroCalibVal[Y_AXIS_INDEX];
//				gyroAngleValues[Y_AXIS_INDEX] = (calc2/calc1)+gyroAngleValues[Y_AXIS_INDEX];
//    		}
//			previousTime = currentTime;
//
//			i++;
//			if(i==20)
//			{
////				uart_printf("gyro: %.1f  accel: %.1f\n", gyroAngleValues[X_AXIS_INDEX], accelAngleValues[X_AXIS_INDEX]);
////				uart_printf("x: %d  y: %d  z: %d\n", accelRawData[X_AXIS_INDEX], accelRawData[Y_AXIS_INDEX], accelRawData[Z_AXIS_INDEX]);
//				uart_printf("x: %.1f  y: %.1f  z: %.1f\n", gyroAngleValues[X_AXIS_INDEX], gyroAngleValues[Y_AXIS_INDEX], gyroAngleValues[Z_AXIS_INDEX]);
//
//				i=0;
//			}
//
//    	}
//	}
//}


void motion_control_task(void *pvParameters)
{
	double accelAngleValues[3]={0};
	uint8_t accelCalibFlag=0;
	double accelCalibVal[3]={0};
	float gyroRadPerSec_X = 0;
	float gyroRadPerSec_Y = 0;
	float gyroRadPerSec_Z = 0;

	double gyroAngleValues[3]={0};
	uint8_t gyroCalibFlag=0;
	double gyroCalibVal[3]={0};
	double calc1;
	double calc2;

	uint32_t previousTime=0;
	uint32_t currentTime=0;
	uint32_t elapsed_time_in_seconds=0;
	extern volatile float q0, q1, q2, q3;
	extern volatile float roll, pitch, yaw;


	uint32_t i=0,j=0;

	while(1)
	{
    	if(!gyroCalibFlag)
    	{
    		uart_printf("gyro calibration started\n");
    		if(gyro_do_calibration(gyroCalibVal)<0)
    		{
    			uart_printf("gyro calibration fail\n");
    			while(1);
    		}
    		gyroCalibFlag = 1;
//    		uart_printf("calib value: %.1f\n", gyroCalibVal[X_AXIS_INDEX]);
    	}
    	else if(!accelCalibFlag)
		{
			uart_printf("accel calibration started\n");
			if(accel_do_calibration(accelCalibVal)<0)
			{
				uart_printf("accel calibration fail\n");
				while(1);
			}
			accelCalibFlag = 1;
//    		uart_printf("calib value: %.1f\n", gyroCalibVal[X_AXIS_INDEX]);
		}

    	else if((gyroCalibFlag == 1)&&(accelCalibFlag == 1))
    	{
    		if(accel_measurement_read(accelRawData)<0)
    			uart_printf("accel read fail\n");
    		accelRawData[X_AXIS_INDEX] = accelRawData[X_AXIS_INDEX]+accelCalibVal[X_AXIS_INDEX];
			accelRawData[Y_AXIS_INDEX] = accelRawData[Y_AXIS_INDEX]+accelCalibVal[Y_AXIS_INDEX];
			accelRawData[Z_AXIS_INDEX] = accelRawData[Z_AXIS_INDEX]+accelCalibVal[Z_AXIS_INDEX];

			vTaskDelay(5/portTICK_PERIOD_MS);

			if(gyro_measurement_read(gyroRawData)<0)
			    uart_printf("gyro read fail\n");
			gyroRawData[X_AXIS_INDEX] = (gyroRawData[X_AXIS_INDEX])-gyroCalibVal[X_AXIS_INDEX];
			gyroRawData[Y_AXIS_INDEX] = (gyroRawData[Y_AXIS_INDEX])-gyroCalibVal[Y_AXIS_INDEX];
			gyroRawData[Z_AXIS_INDEX] = (gyroRawData[Z_AXIS_INDEX])-gyroCalibVal[Z_AXIS_INDEX];

			//convert to radians/second
			gyroRadPerSec_X = (((float)gyroRawData[X_AXIS_INDEX])*M_PI)/180;
			gyroRadPerSec_Y = (((float)gyroRawData[Y_AXIS_INDEX])*M_PI)/180;
			gyroRadPerSec_Z = (((float)gyroRawData[Z_AXIS_INDEX])*M_PI)/180;

			for(j=0;j<20;j++)
			{
				MadgwickAHRSupdateIMU(gyroRadPerSec_X, gyroRadPerSec_Y, gyroRadPerSec_Z,
						(float)accelRawData[X_AXIS_INDEX], (float)accelRawData[Y_AXIS_INDEX], (float)accelRawData[Z_AXIS_INDEX]);
				Madgwick_computeAngles();
			}

			i++;
			if(i==10)
			{
//				uart_printf("q0: %.1f  q1: %.1f  q2: %.1f  q3: %.1f\n", q0, q1, q2, q3);
				uart_printf("roll: %.1f  pitch: %.1f  yaw: %.1f\n", roll*DEGREE_CNVRT_CONST, pitch*DEGREE_CNVRT_CONST, yaw*DEGREE_CNVRT_CONST);
//				uart_printf("x: %.1f  y: %.1f  z: %.1f\n", (float)gyroRawData[X_AXIS_INDEX], (float)gyroRawData[Y_AXIS_INDEX], (float)gyroRawData[Z_AXIS_INDEX]);

				i=0;
			}

    	}
	}
}


void drone_init_task(void *pvParameters)
{
	timer3_init();
	motors_pwm_init();
	I2C_LowLevel_Init(400000, 0x38);
	debug_led_init();
	uart_console_init(9600);
	/*safety delay for mpu6050 to powerup*/
	vTaskDelay(500/portTICK_PERIOD_MS);
	if(mpu6050_init(FS_SEL3, FS_SEL1)<0)
	{
		uart_printf("mpu6050 init failed.\n");
		while(1);
	}
	else
	{
		uart_printf("drone init complete.\n");
		vTaskDelay(2000/portTICK_PERIOD_MS);
		xTaskCreate(motion_control_task, "motion_control_task", 500, NULL, 1, NULL );
		vTaskDelete(NULL);
	}
}

int main(void)
{
	xTaskCreate(drone_init_task, "drone_init_task", 200, NULL, 0, NULL );
	vTaskStartScheduler();
	while(1);

    while(1)
    {


//    	motor_pwm_speed_set(PWM_CHANNEL4, 5);
//    	delay_ms(1000);
//    	motor_pwm_speed_set(PWM_CHANNEL4, 10);
//    	delay_ms(1000);

//    	data = mpu6050_register_read(0x75);
//    	uart_printf("data %d\n", data);
//    	delay_ms(2000);

//    	timeout_alarm_set(200);
//    	while(!timeout_alarm_status_check());
//    	gpio_val = !gpio_val;
//    	GPIO_WriteBit(GPIOC, GPIO_Pin_8, gpio_val);

    }
}

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; );
}

void vApplicationIdleHook( void )
{
	volatile size_t xFreeStackSpace;

	/* This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amout of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
