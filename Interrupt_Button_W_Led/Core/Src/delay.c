/*
 * delay.c
 *
 *  Created on: 09-Jul-2022
 *      Author: rajdeep
 */

#include "delay.h"

void TIM6Config (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable Timer clock
	2. Set the prescalar and the ARR
	3. Enable the Timer, and wait for the update Flag to set
	************************************************/

// 1. Enable Timer clock
	RCC->APB1ENR |= (1<<4);  // Enable the timer6 clock

// 2. Set the prescalar and the ARR
	TIM6->PSC =	1999;  // 45MHz/45 = 1 MHz ~~ 1 uS delay
	TIM6->ARR = 1599;  // MAX ARR value
	TIM6->CNT = 0;

// 3. Enable the Timer, and wait for the update Flag to set
	TIM6->CR1 |= (1<<0); // Enable the Counter
	while (!(TIM6->SR & (1<<0)));  // UIF: Update interrupt flag..  This bit is set by hardware when the registers are updated
}


void Delay_us (uint32_t us)
{
	/************** STEPS TO FOLLOW *****************
	1. RESET the Counter
	2. Wait for the Counter to reach the entered value. As each count will take 1 us,
		 the total waiting time will be the required us delay
	************************************************/
	TIM6->CNT = 0;
	while (TIM6->CNT < us);
}


void Delay_ms (uint32_t ms)
{
	for (uint16_t i=0; i<ms; i++)
	{
		Delay_us (1000); // delay of 1 ms
	}
}


