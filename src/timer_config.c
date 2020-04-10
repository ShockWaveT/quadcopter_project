/**
 * @file 	timer_config.c
 * @date 	08-Jan-2020
 * @author 	Arun Cheriyan
 */
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "timer_config.h"
#include "stm32f10x_tim.h"

volatile uint32_t elapsed_ms=0;
volatile uint8_t timeOutTimerStatus=0;
volatile uint32_t timeOutTimerAlarmTime=0;
volatile uint32_t timeOutTimerStartTime=0;
volatile uint8_t timeOutTimerAlarmStatus=0;

/************************************************************************//*
 * Timer 3 ISR
 *
 * @return none.
 **************************************************************************/
void TIM3_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
    	elapsed_ms++;
    	if(timeOutTimerStatus == 1)
    	{
    		if((elapsed_ms - timeOutTimerStartTime) > timeOutTimerAlarmTime)
    			timeOutTimerAlarmStatus = 1;
    	}
   }

    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}


/************************************************************************//*
 * Initializes timer 3 peripheral for delay and millis functions.
 *
 * @return none.
 **************************************************************************/
void timer3_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//configure timer 3 interrupt
	NVIC_InitTypeDef nvicStructure;
	nvicStructure.NVIC_IRQChannel = TIM3_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

	/*
	 * timer 3 is run by clock of 1000Hz. to get this clock we
	 * need to divide system clock(24MHz) with 24000.
	 * In this way timer counter increments by one every 1ms.
	 *
	 * eg: to generate an interrupt every second we can load
	 * (1000-1) to 'TIM_Period'. so counter counts to
	 * (1000-1) and resets back to zero and also generates an
	 * interrupt.
	 *
	 * TIM_Period is actually auto reload register(ARR).
	 */
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 2400-1;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 10-1;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &timerInitStructure);

	TIM_ITConfig(TIM3, TIM_IT_Update , ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}


/************************************************************************//*
 * initializes timer 2 to output PWM signals
 *
 * @return none.
 **************************************************************************/
void motors_pwm_init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* GPIOA and GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = MOTOR_TIM_ARR_REG_VAL;
	TIM_TimeBaseStructure.TIM_Prescaler = MOTOR_TIM_PRESCALER_VAL;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	/* TIM3 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}


/************************************************************************//*
 * sets the pwm duty cycle for specified channel
 *
 * @param channelID: channel ID.
 * this can be PWM_CHANNEL1, PWM_CHANNEL2, PWM_CHANNEL3, PWM_CHANNEL4
 * @param speedPercentage: duty cycle in percentage value(0-100)
 * @return none.
 **************************************************************************/
void motor_pwm_speed_set(pwm_channels_t channelID, float speedPercentage)
{
	uint32_t pwm_value;

	pwm_value = (speedPercentage*MOTOR_TIM_ARR_REG_VAL)/100;
	if(channelID == PWM_CHANNEL1)
		TIM_SetCompare1(TIM2, pwm_value);
	if(channelID == PWM_CHANNEL2)
		TIM_SetCompare2(TIM2, pwm_value);
	if(channelID == PWM_CHANNEL3)
		TIM_SetCompare3(TIM2, pwm_value);
	if(channelID == PWM_CHANNEL4)
		TIM_SetCompare4(TIM2, pwm_value);
}


/************************************************************************//*
 * for tracking the current time
 *
 * @retval current time in milliseconds
 **************************************************************************/
uint32_t millis(void)
{
	return elapsed_ms;
}


/************************************************************************//*
 * produces delay in the program by continuously looping. returns when
 * given time has elapsed.
 *
 * @param delay_ms: delay time in milli seconds
 * @return none.
 **************************************************************************/
void delay_ms(uint32_t delay_ms)
{
	uint32_t currentTime_ms;
	uint32_t startTime_ms;

	startTime_ms = millis();
	while(1)
	{
		currentTime_ms = millis();
		if((currentTime_ms-startTime_ms)>delay_ms)
			return ;
	}
}


/************************************************************************//*
 * sets an alarm to so that timeouts can be implemented
 *
 * @param alarmTime_ms: timeout time in milliseconds.
 * @return none.
 **************************************************************************/
void timeout_alarm_set(uint32_t alarmTime_ms)
{
	timeOutTimerStartTime = millis();
	timeOutTimerStatus = 1;
	timeOutTimerAlarmTime = alarmTime_ms;
	timeOutTimerAlarmStatus = 0;
}


/************************************************************************//*
 * checks if timeout has occurred
 *
 * retval 1 if timeout happened. else 0.
 **************************************************************************/
uint8_t timeout_alarm_status_check(void)
{
	return timeOutTimerAlarmStatus;
}


/************************************************************************//*
 * turns off the alarm.
 *
 * @return none.
 **************************************************************************/
void timeout_alarm_off(void)
{
	timeOutTimerStatus = 0;
	timeOutTimerAlarmStatus = 0;
}


/************************************************************************//*
 *
 **************************************************************************/
uint8_t timeout_wait(uint32_t* timeoutVar, uint32_t timeOut_ms)
{
	static uint32_t startTime_ms;

	if((*timeoutVar)>0)
	{
		(*timeoutVar) = millis()-startTime_ms;
		if((*timeoutVar)>timeOut_ms)
			return 0;
		else
			return 1;
	}
	else
	{
		startTime_ms = millis();
		delay_ms(1);
		(*timeoutVar) = 1;
		return 1;
	}

}
