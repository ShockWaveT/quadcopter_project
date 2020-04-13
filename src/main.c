/**
 * @file 	main.c
 * @date 	09-Feb-2020
 * @author 	Arun Cheriyan
 */
#include <math.h>
#include <motion_control_algorithms/imu_utils.h>
#include <motion_control_algorithms/MadgwickAHRS.h>
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "i2c_comm.h"
#include "small_printf.h"
#include "timer_config.h"
#include "gy87_module.h"
#include "uart_comm.h"
#include "motors.h"
#include "debug.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


int16_t gyroRawData[3];
int16_t accelRawData[3];
int16_t magRawData[3];

void bluetooth_cmd_task(void *pvParameters)
{
	uint16_t data=0;

	uart_printf("listening for uart...\n");
	while(1)
	{
		while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == 0);
		data = USART_ReceiveData(USART1);
		uart_printf("data: %c\n", data);

	}
}

void test_task(void *pvParameters)
{
	int8_t errorValue;
	float pidOutput;

	int16_t magMax[3];
	int16_t magMin[3];
	extern volatile float roll, pitch, yaw;

	vTaskDelay(8000/portTICK_PERIOD_MS);
	while(1)
	{
//		servo_set_position(PWM_CHANNEL4, 0);
//		delay_ms(1000);
//		servo_set_position(PWM_CHANNEL4, 180);
//		delay_ms(1000);

		pidOutput = pid_compute(roll, 0);
		uart_printf("%.1f\n", pidOutput);
		vTaskDelay(100/portTICK_PERIOD_MS);


//		errorValue++;
//		if(errorValue == 100000)
//			uart_printf("test task\n");


//		uart_printf("start moving the magnetometer\n");
//
//		if(magnetometer_read_min_and_max_values(magMax, magMin) < 0)
//			uart_printf("mag error during calibration\n");
//		else
//			uart_printf("x_offset: %d  y_offset: %d  z_offset: %d\n",
//						(magMax[X_AXIS_INDEX]+magMin[X_AXIS_INDEX])/2,
//						(magMax[Y_AXIS_INDEX]+magMin[Y_AXIS_INDEX])/2,
//						(magMax[Z_AXIS_INDEX]+magMin[Z_AXIS_INDEX])/2);
//
//		uart_printf("x_max: %d  x_min: %d  y_max: %d  y_min: %d   z_max: %d  z_min: %d\n",
//					magMax[X_AXIS_INDEX], magMin[X_AXIS_INDEX],
//					magMax[Y_AXIS_INDEX], magMin[Y_AXIS_INDEX],
//					magMax[Z_AXIS_INDEX], magMin[Z_AXIS_INDEX]);
//
//		vTaskDelay(60000/portTICK_PERIOD_MS);

	}
}

void motion_control_task(void *pvParameters)
{
#define CALC_WORK_TIME 0
#define DO_AUTOMATIC_MAG_CALIB 0

#if CALC_WORK_TIME
uint8_t workCount=0;
#endif

	uint8_t accelCalibCompleteFlag=0;
	uint8_t gyroCalibCompleteFlag=0;
	uint8_t magCalibCompleteFlag=0;
	float accelCalibVal[3]={0};
	float gyroCalibVal[3]={0};
	int16_t magnetometerMax[3]={94, 199, 156};
	int16_t magnetometerMin[3]={-255, -200, -174};

	float accelFiltered[3]={0};
	float gyroRawRadPerSec[3] = {0};

	extern volatile float roll, pitch, yaw;
	const float accelFilterAlpha = 0.05;

	uint32_t i=0,j=0;
	uint8_t magReadCount=0;


	if(!gyroCalibCompleteFlag)
	{
		uart_printf("gyro calibration started\n");
		if(gyro_calc_bias(gyroCalibVal)<0)
		{
			uart_printf("gyro calibration fail\n");
			while(1);
		}
		gyroCalibCompleteFlag = 1;
		vTaskDelay(10/portTICK_PERIOD_MS);
	}

	if(!accelCalibCompleteFlag)
	{
		uart_printf("accel calibration started\n");
		if(accel_calc_bias(accelCalibVal)<0)
		{
			uart_printf("accel calibration fail\n");
			while(1);
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
		accelCalibCompleteFlag = 1;
	}

#if DO_AUTOMATIC_MAG_CALIB
	if(!magCalibCompleteFlag)
	{
		uart_printf("start moving the magnetometer\n");
		if(magnetometer_read_min_and_max_values(magnetometerMax, magnetometerMin) < 0)
		{
			uart_printf("mag error during calibration\n");
			while(1);
		}
		uart_printf("magnetometer calibration complete\n");
		vTaskDelay(10/portTICK_PERIOD_MS);
		magCalibCompleteFlag = 1;
	}
#else
	magCalibCompleteFlag = 1;
#endif

	while(1)
	{
    	if((gyroCalibCompleteFlag == 1)&&(accelCalibCompleteFlag == 1)&&
    	   (magCalibCompleteFlag == 1))
    	{
#if CALC_WORK_TIME
if(workCount==0)
	uart_printf("*%d\n",millis());
#endif

    		if(magReadCount>=2)
			{
				if(magneto_measurement_read(magRawData)<0)
					uart_printf("magnetometer read fail\n");
				magReadCount=0;
				vTaskDelay(1/portTICK_PERIOD_MS);

			}
			magReadCount++;

    		if(imu_raw_measurement_read(accelRawData, gyroRawData)<0)
    			uart_printf("imu read fail\n");

    		accel_caliberate(accelRawData, accelCalibVal);

    		low_pass_filter(accelRawData[X_AXIS_INDEX], &accelFiltered[X_AXIS_INDEX], accelFilterAlpha);
    		low_pass_filter(accelRawData[Y_AXIS_INDEX], &accelFiltered[Y_AXIS_INDEX], accelFilterAlpha);
    		low_pass_filter(accelRawData[Z_AXIS_INDEX], &accelFiltered[Z_AXIS_INDEX], accelFilterAlpha);

    		gyro_caliberate(gyroRawData, gyroCalibVal);

			//convert to radians/second
    		gyroRawRadPerSec[X_AXIS_INDEX] = convert_degrees_to_radians((float)gyroRawData[X_AXIS_INDEX]);
    		gyroRawRadPerSec[Y_AXIS_INDEX] = convert_degrees_to_radians((float)gyroRawData[Y_AXIS_INDEX]);
    		gyroRawRadPerSec[Z_AXIS_INDEX] = convert_degrees_to_radians((float)gyroRawData[Z_AXIS_INDEX]);

    		magnetometer_caliberate(magRawData, magnetometerMax, magnetometerMin);

			for(j=0;j<5;j++)
			{
				MadgwickAHRSupdate(gyroRawRadPerSec[X_AXIS_INDEX], gyroRawRadPerSec[Y_AXIS_INDEX], gyroRawRadPerSec[Z_AXIS_INDEX],
								   accelFiltered[X_AXIS_INDEX], accelFiltered[Y_AXIS_INDEX], accelFiltered[Z_AXIS_INDEX],
								   magRawData[X_AXIS_INDEX], magRawData[Y_AXIS_INDEX], magRawData[Z_AXIS_INDEX]);
			}
			Madgwick_computeAngles();

#if CALC_WORK_TIME
workCount++;
if(workCount == 10)
{
	uart_printf("#%d\n",millis());
	workCount=0;
}
#endif

			i++;
			if(i==20)
			{
//				uart_printf("%.1f\n", convert_radians_to_degrees(yaw));
				i=0;
			}

    	}
	}
}


void drone_init_task(void *pvParameters)
{
	int32_t errorCode=0;

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

	if(mpu6050_aux_i2c_bus_host_access(MPU6050_HOST_AUX_BUS_CONNECT)<0)
	{
		uart_printf("aux bus connect failed.\n");
		while(1);
	}

	if(hmc5883l_init()<0)
	{
		uart_printf("magnetometer init failed.\n");
		while(1);
	}

	else
	{
		uart_printf("drone init complete.\n");
		vTaskDelay(2000/portTICK_PERIOD_MS);
		errorCode = xTaskCreate(test_task, "test_task", 200, NULL, 1, NULL );
		uart_printf("task1:%d\n", errorCode);
		errorCode = xTaskCreate(motion_control_task, "motion_control_task", 500, NULL, 1, NULL );
		uart_printf("task2:%d\n", errorCode);

		vTaskDelete(NULL);
	}
}

int main(void)
{
  	xTaskCreate(drone_init_task, "drone_init_task", 200, NULL, 0, NULL );
  	xTaskCreate(bluetooth_cmd_task, "bluetooth command task", 200, NULL, 0, NULL );
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
