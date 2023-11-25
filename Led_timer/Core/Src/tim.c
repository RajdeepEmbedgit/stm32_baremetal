/*
 * tim.c
 *
 *  Created on: 21-Jun-2022
 *      Author: Admin
 */

#include "stm32f4xx.h"

#define TIM2EN					(1U<<0)
#define CR1_CEN					(1U<<0)


void tim2_1hz_init(void)
{
	RCC->APB1ENR |= TIM2EN ;
	TIM2->PSC = 1599;		//default	1600
	TIM2->ARR =	9999;				 //14999;	// 1MIN		//44999;	FOR 3MIN		//default	10000
	TIM2->CNT = 0;

	TIM2->CR1 = CR1_CEN;
}

void start_Tim2 (void)
{
	while(!(TIM2->SR & TIM_SR_UIF)){}

	TIM2->SR &=~ TIM_SR_UIF;
}

