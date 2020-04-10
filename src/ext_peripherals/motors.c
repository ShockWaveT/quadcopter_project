/**
 * @file 	motors.c
 * @date 	10-Apr-2020
 * @author 	Arun Cheriyan
 */
#include "stm32f10x.h"
#include "small_printf.h"
#include "timer_config.h"


void servo_set_position(pwm_channels_t channelID, uint8_t degreeVal)
{


	motor_pwm_speed_set(channelID, 5);

}
