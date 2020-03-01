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

int16_t gyroRawData[3];
int16_t accelRawData[3];


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
//	double accelRaw_x;
//	double accelRaw_y;
//	double accelRaw_z;
//	double accelAngle_x;
//	double accelAngle_y;
//
//	double gyroSum=0;
//	int32_t gyroAngle;
//	double calc1;
//	double calc2;
//
//	uint32_t previousTime=0;
//	uint32_t currentTime=0;
//	uint32_t elapsed_time_in_seconds=0;
//
//	while(1)
//	{
//		if(accel_measurement_read(accelData)<0)
//			uart_printf("accel read fail\n");
//
//		vTaskDelay(25/portTICK_PERIOD_MS);
//
//		if(gyro_measurement_read(gyroData)<0)
//		    uart_printf("gyro read fail\n");
//
//		accelRaw_x = (double)(accelData[0]);
//		accelRaw_y = (double)(accelData[1]);
//		accelRaw_z = (double)(accelData[2]);
//		accelAngle_x = atan( accelRaw_x/sqrt((accelRaw_z*accelRaw_z)+(accelRaw_y*accelRaw_y)) );
//		accelAngle_y = atan( accelRaw_y/sqrt((accelRaw_z*accelRaw_z)+(accelRaw_x*accelRaw_x)) );
//
//
//		currentTime = millis();
//    	elapsed_time_in_seconds = (1000/(currentTime-previousTime));
//
//    	calc1 = gyroData[0]+40;
//    	calc2 = elapsed_time_in_seconds*(32.8f);
//    	gyroSum = (calc1/calc2)+gyroSum;
//    	previousTime = currentTime;
//
//
////		uart_printf("x: %.1f    y: %.1f    z:%.1f \n",
////				   (accelAngle_x*180)/3.14, (accelAngle_y*180)/3.14, (accelAngle_z*180)/3.14);
//    	uart_printf("x: %.1f \n",gyroSum);
////		delay_ms(50);
//    	vTaskDelay(25/portTICK_PERIOD_MS);
//	}
//}

//void motion_control_task(void *pvParameters)
//{
//	double accelRaw_x;
//	double accelRaw_y;
//	double accelRaw_z;
//	double accelAngle_x;
//	double accelAngle_y;
//
//	double gyroSum=0;
//	int32_t gyroAngle;
//	double calc1;
//	double calc2;
//
//	uint32_t previousTime=0;
//	uint32_t currentTime=0;
//	uint32_t elapsed_time_in_seconds=0;
//
//	uint8_t caliberateFlag=0;
//	uint32_t caliberateCount=0;
//	double caliberateAvg=0;
//
//
//	while(1)
//	{
//		if(gyro_measurement_read(gyroData)<0)
//		    uart_printf("gyro read fail\n");
//
//    	if(!caliberateFlag)
//    	{
//			if(caliberateCount<200)
//			{
//				caliberateAvg = (gyroData[0]+caliberateAvg);
//				caliberateCount++;
//			}
//			else
//			{
//				caliberateAvg = (caliberateAvg/200);
////				if(caliberateAvg<0)
////					caliberateAvg = caliberateAvg-1;
////				if(caliberateAvg>0)
////					caliberateAvg = caliberateAvg+1;
//				caliberateFlag=1;
//				uart_printf("Calibration complete: %.1f\n",caliberateAvg);
//			}
//    	}
//
//    	else if(caliberateFlag)
//    	{
//    		currentTime = millis();
//    		elapsed_time_in_seconds = (1000/(currentTime-previousTime));
//
//    		if(elapsed_time_in_seconds!=0)
//    		{
//				calc1 = (double)gyroData[0]-caliberateAvg;
//				calc2 = (double)elapsed_time_in_seconds*(32.8f);
//				gyroSum = (calc1/calc2)+gyroSum;
//    		}
//			previousTime = currentTime;
//
////				uart_printf("raw x: %d     x: %.1f \n",gyroData[0], gyroSum);
//			uart_printf("calc1: %.1f   calc2: %.1f   gyroSum: %.1f\n",calc1, calc2, gyroSum);
//    	}
//    	vTaskDelay(50/portTICK_PERIOD_MS);
//	}
//}

void motion_control_task(void *pvParameters)
{
	double accelAngleValues[3]={0};
	double gyroAngleValues[3]={0};
	double calc1;
	double calc2;

	uint32_t previousTime=0;
	uint32_t currentTime=0;
	uint32_t elapsed_time_in_seconds=0;

	uint8_t caliberateFlag=0;
	double gyroCalibVal[3]={0};

	uint32_t i=0;


	while(1)
	{
		if(gyro_measurement_read(gyroRawData)<0)
		    uart_printf("gyro read fail\n");

    	if(!caliberateFlag)
    	{
    		uart_printf("calibration started\n");
    		if(gyro_do_calibration(gyroCalibVal)<0)
    		{
    			uart_printf("calibration fail\n");
    			while(1);
    		}
    		caliberateFlag = 1;
//    		uart_printf("calib value: %.1f\n", gyroCalibVal[X_AXIS_INDEX]);
    	}

    	else if(caliberateFlag)
    	{
    		currentTime = millis();
    		elapsed_time_in_seconds = (1000/(currentTime-previousTime));

    		if(elapsed_time_in_seconds!=0)
    		{
    			calc1 = ((double)elapsed_time_in_seconds)*16.4f;

				calc2 = ((double)gyroRawData[X_AXIS_INDEX])-gyroCalibVal[X_AXIS_INDEX];
				gyroAngleValues[X_AXIS_INDEX] = (calc1/calc2)+gyroAngleValues[X_AXIS_INDEX];

				calc2 = (double)gyroRawData[Y_AXIS_INDEX]-gyroCalibVal[Y_AXIS_INDEX];
				gyroAngleValues[Y_AXIS_INDEX] = (calc2/calc1)+gyroAngleValues[Y_AXIS_INDEX];
    		}
			previousTime = currentTime;

//				uart_printf("raw x: %d     x: %.1f \n",gyroData[0], gyroSum);
			i++;
			if(i==20)
			{
				uart_printf("gyro angle: %.1f\n", gyroAngleValues[Y_AXIS_INDEX]);
				i=0;
			}
    	}
    	vTaskDelay(5/portTICK_PERIOD_MS);
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
