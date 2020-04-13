/*
 * timer_config.h
 *
 *  Created on: 08-Jan-2020
 *      Author: arun
 */

#ifndef TIMER_CONFIG_H_
#define TIMER_CONFIG_H_

#define MOTOR_TIM_ARR_REG_VAL	(600-1)//200-1
#define MOTOR_TIM_PRESCALER_VAL	(800-1)//


typedef enum
{
	PWM_CHANNEL1=1,
	PWM_CHANNEL2=2,
	PWM_CHANNEL3=3,
	PWM_CHANNEL4=4,
}pwm_channels_t;


void timer3_init(void);
void motors_pwm_init(void);
void pwm_output_set(uint8_t channelID, float speedPercentage);
uint32_t millis(void);
void delay_ms(uint32_t delay_ms);
uint8_t timeout_wait(uint32_t* timeoutVar, uint32_t timeOut_ms);
void timeout_alarm_set(uint32_t alarmTime_ms);
uint8_t timeout_alarm_status_check(void);
void timeout_alarm_off(void);

#endif /* TIMER_CONFIG_H_ */
