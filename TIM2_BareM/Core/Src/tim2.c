/*
 * tim2.c
 *
 *  Created on: Jun 22, 2022
 *      Author: Admin
 */
#include "stm32f4xx.h"

uint16_t	cnt = 0;


#define TIM2EN					(1U<<0)
#define CR1_CEN					(1U<<0)


void HART_Tim2_init (void)
{
		RCC->APB1ENR |= TIM2EN ;
		TIM2->PSC = 89;			//4999;		//default	1600
		TIM2->ARR =	100000-1;			//17999;				 //14999;	// 1MIN		//44999;	FOR 3MIN		//default	10000
		TIM2->CNT = 0;

		TIM2->CR1 = CR1_CEN;
}


void start_Tim2 (void)
{
	while(!(TIM2->SR & TIM_SR_UIF)){}

	TIM2->SR &=~ TIM_SR_UIF;

	cnt++;
}


