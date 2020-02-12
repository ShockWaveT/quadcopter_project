#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "i2c_comm.h"
#include "small_printf.h"
#include "timer_config.h"
#include "gy87_module.h"

int16_t gyroData[3];
int32_t gyroSum=0;
int32_t gyroAngle;
uint32_t previousTime=0;
uint32_t currentTime=0;
uint32_t elapsed_time_in_seconds=0;

void uart_console_init(uint32_t baudRate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable peripheral clocks for USART1 on GPIOA */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA |
						   RCC_APB2Periph_AFIO, ENABLE);

	/* Configure PA9 and PA10 as USART1 TX/RX */

	/* PA9 = alternate function push/pull output */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* PA10 = floating input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure and initialize usart... */
	USART_InitStructure.USART_BaudRate = baudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE);
}

void debug_led_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Enable GPIO clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	// Configure pin as output push-pull (LED)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}


int main(void)
{
	uint8_t data=0;
	uint8_t gpio_val;
	uint32_t k=0;

	int32_t calc1=0;
	int32_t calc2=0;


	timer3_init();
	motors_pwm_init();
	I2C_LowLevel_Init(400000, 0x38);
	debug_led_init();
	uart_console_init(9600);
	delay_ms(2000);
	uart_printf("start...\n");
	if(gyro_init(0x10)<0)
	{
		uart_printf("fail1...\n");
		while(1);
	}


    while(1)
    {
    	if(gyro_measurement_read(gyroData)<0)
    		uart_printf("fail2...\n");
    	currentTime = millis();
    	elapsed_time_in_seconds = (1000/(currentTime-previousTime));

    	calc1 = (gyroData[0]+26)*100000;
    	calc2 = elapsed_time_in_seconds*(32.8f);
    	gyroSum = (calc1/calc2)+gyroSum;
    	previousTime = currentTime;

    	k++;
    	if(k==1000)
    	{
    		uart_printf("x_angle: %d\n",(int32_t)(gyroSum/100000));
//    		uart_printf("calc1: %d\n", (int32_t)calc1);
    		k=0;
    	}

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
