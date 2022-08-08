/******************************************************************************
 *
 * Module: SW PWM
 *
 * File Name: AVELABS
 *
 * Author: Ahmed Atef
 *
 *******************************************************************************/

#include"pwm.h"
#include "gpio.h"
#include "common_macros.h" /* To use the macros like SET_BIT */
#include "avr/io.h" /* To use the IO Ports Registers */
#include <avr/io.h>
#include <util/delay.h>

/* global variable contain the ticks count of the timer */
unsigned char g_tick=0;

ISR(TIMER0_OVF_vect)
{
	g_tick++;
}
void GPIO_INIT(uint8 pinNumber, uint8 port, GPIO_PinDirectionType direction){
	if((pinNumber >= NUM_OF_PINS_PER_PORT) || (pinNumber >= NUM_OF_PORTS))
		{
			/* Do Nothing */
		}
		else
		{
			/* Setup the pin direction as required */
			switch(port)
			{
			case PORTA_ID:
				if(direction == PIN_OUTPUT)
				{
					SET_BIT(DDRA,pinNumber);
				}
				else
				{
					CLEAR_BIT(DDRA,pinNumber);
				}
				break;
			}
		}
}

void GPIO_writePin(uint8 port_num, uint8 pin_num, uint8 value){
	if((pin_num >= NUM_OF_PINS_PER_PORT) || (port_num >= NUM_OF_PORTS))
		{
			/* Do Nothing */
		}
		else
		{
			/* Write the pin value as required */
			switch(port_num)
			{
			case PORTA_ID:
				if(value == LOGIC_HIGH)
				{
					SET_BIT(PORTA_ID,pin_num);
				}
				else
				{
					CLEAR_BIT(PORTA_ID,pin_num);
				}
				break;
			}
		}
}

uint8 GPIO_readPin(uint8 port_num, uint8 pin_num)
{
	uint8 pin_value = LOGIC_LOW;

	/*
	 * Check if the input port number is greater than NUM_OF_PINS_PER_PORT value.
	 * Or if the input pin number is greater than NUM_OF_PINS_PER_PORT value.
	 * In this case the input is not valid port/pin number
	 */
	if((pin_num >= NUM_OF_PINS_PER_PORT) || (port_num >= NUM_OF_PORTS))
	{
		/* Do Nothing */
	}
	else
	{
		/* Read the pin value as required */
		switch(port_num)
		{
		case PORTA_ID:
			if(BIT_IS_SET(port_num,pin_num))
			{
				pin_value = LOGIC_HIGH;
			}
			else
			{
				pin_value = LOGIC_LOW;
			}
			break;
		}
	}
	return pin_value;
}

void PWM_Timer0_Start(uint8 set_duty_cycle)
{

	TCNT0 = 0; //Set Timer Initial value

	OCR0  = set_duty_cycle; // Set Compare Value

	DDRB  = DDRB | (1<<PB3); //set PB3/OC0 as output pin --> pin where the PWM signal is generated from MC.

	/* Configure timer control register
	 * 1. Fast PWM mode FOC0=0
	 * 2. Fast PWM Mode WGM01=1 & WGM00=1
	 * 3. Clear OC0 when match occurs (non inverted mode) COM00=0 & COM01=1
	 * 4. clock = F_CPU/8 CS00=0 CS01=1 CS02=0
	 */
	TCCR0 = (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS00);
}

void PWM_Timer0_Changeduty(uint8 new_duty_cycle)
{
	OCR0 = new_duty_cycle;
}

void Timer_Start(void)
{
	TCNT0 = 0; // Set Timer initial value to 0
	TIMSK = (1<<TOIE0); // Enable Timer0 Overflow Interrupt
	/* configure the timer
	 * 1. Non PWM mode FOC0=1
	 * 2. Normal Mode WGM01=0 & WGM00=0
	 * 3. Normal Mode COM00=0 & COM01=0
	 * 4. clock = F_CPU/256 CS00=0 CS01=0 CS02=1
	 */
	TCCR0 = (1<<FOC0) | (1<<CS02);
}

void Timer_Stop(stop)
{
 g_tick=0;
}
