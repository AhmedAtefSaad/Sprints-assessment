 /******************************************************************************
 *
 * Module: PWM
 *
 * File Name: pwm.h
 *
 * Description: header file for the ATmega16/32 PWM driver
 *
 * Author: Ahmed Atef
 *
 *******************************************************************************/

#ifndef PWM_H_
#define PWM_H_

#include "std_types.h"

/*
 * Description :
 * Function responsible for generating PWM signal on  OC0/PB3
 */
void PWM_Timer0_Start(uint8 set_duty_cycle);

#endif /* PWM_H_ */
