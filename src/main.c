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

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

int16_t gyroData[3];
int16_t accelData[3];
int32_t gyroSum=0;
int32_t gyroAngle;
double accel_x;
double accel_y;
double accel_z;
double accelAngle;

uint32_t previousTime=0;
uint32_t currentTime=0;
uint32_t elapsed_time_in_seconds=0;


void test_task(void *pvParameters)
{
	while(1)
	{
		uart_printf("testing task\n");
		vTaskDelay(1000/portTICK_PERIOD_MS);
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
	if(mpu6050_init(FS_SEL2, FS_SEL1)<0)
	{
		uart_printf("mpu6050 init failed.\n");
		while(1);
	}
	else
	{
		uart_printf("drone init complete.\n");
		xTaskCreate(test_task, "test_task", 1000, NULL, 0, NULL );
		vTaskDelete(NULL);
	}
}

int main(void)
{
	uint8_t data=0;
	uint8_t gpio_val;
	uint32_t k=0;

	int32_t calc1=0;
	int32_t calc2=0;

	xTaskCreate(drone_init_task, "drone_init_task", 500, NULL, 0, NULL );
	vTaskStartScheduler();
	while(1);

    while(1)
    {




//    	if(gyro_measurement_read(gyroData)<0)
//    		uart_printf("fail2...\n");
//    	currentTime = millis();
//    	elapsed_time_in_seconds = (1000/(currentTime-previousTime));
//
//    	calc1 = (gyroData[0]+26)*100000;
//    	calc2 = elapsed_time_in_seconds*(32.8f);
//    	gyroSum = (calc1/calc2)+gyroSum;
//    	previousTime = currentTime;
//    	k++;
//    	if(k==1000)
//    	{
//    		uart_printf("x_angle: %d\n",(int32_t)(gyroSum/100000));
//    		k=0;
//    	}

    	if(accel_measurement_read(accelData)<0)
    		uart_printf("fail2...\n");
//    	accel_x = (double)(accelData[0]-280)/(double)FS_SEL1_ACCEL_SCALE;
//    	accel_y = (double)(accelData[1]-60)/(double)FS_SEL1_ACCEL_SCALE;
//    	accel_z = (double)(accelData[2]-7380)/(double)FS_SEL1_ACCEL_SCALE;
    	accel_x = (double)(accelData[0]-280);
		accel_y = (double)(accelData[1]-60);
		accel_z = (double)(accelData[2]);

		accelAngle = atan( accel_x/sqrt((accel_z*accel_z)+(accel_y*accel_y)) );
		uart_printf("accel data %f\n", (accelAngle*180)/3.14);
//		uart_printf("accel data %f\n", accel_x);
    	delay_ms(50);

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
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; );
}
/*-----------------------------------------------------------*/

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
