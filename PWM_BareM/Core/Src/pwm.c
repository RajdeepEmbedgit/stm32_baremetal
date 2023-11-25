/*
 * pwm.c
 *
 *  Created on: Jul 7, 2022
 *      Author: rajdeep
 */

#include "pwm.h"



#define TIM4EN					(1U<<2)
#define CR1_CEN					(1U<<0)


void HART_Tim_init (void)
{
		RCC->APB1ENR |= TIM4EN ;
		TIM4->PSC = 1599;		//default	1600
		TIM4->ARR =	14999;				 //14999;	// 1MIN		//44999;	FOR 3MIN		//default	10000
		TIM4->CNT = 0;
		TIM4->CR1 = CR1_CEN;
}


void start_Tim (void)
{
	while(!(TIM4->SR & TIM_SR_UIF)){}

	TIM4->SR &=~ TIM_SR_UIF;
}




void pwm_init(void)
{

		RCC->AHB1ENR |= (1<<0);		//GPIOA SET
		RCC->AHB1ENR |= (1<<1);		//GPIOB SET
		GPIOA->MODER |= (1<<1);		//GPIO 10 :AF USED FOR PWM	SET PA0
		GPIOB->MODER |= (1<<21);		//GPIO 10 :AF USED FOR PWM	SET PB10
		GPIOB->MODER |= (1<<23);		//GPIO 10 :AF USED FOR PWM	SET PB11

		GPIOA->AFR[0] |= 0x00000001;	//SET PIN TO AF1 FOR TIM2_CH1
		GPIOB->AFR[1] |= 0x00000100;	//SET PIN TO AF1 FOR TIM2_CH3
		GPIOB->AFR[1] |= 0x00001000;	//SET PIN TO AF1 FOR TIM2_CH4

		RCC->APB1ENR |= (1<<0);			//TIM2 RCC SET
		TIM2->PSC = 1399;
		TIM2->ARR = 999;		//255

		/*TIM2->CCR1 = 200;			//SET MATCH VALUE 20% = 1000 * 20 / 100
		TIM2->CCR3 = 200;
		TIM2->CCR4 = 200;
*/
		TIM2->CCMR1 = (0x6 << 4);	//SET OUTPUT TO TOGGLE ON MATCH	CH1
		TIM2->CCMR2 = (0x6 << 4);	//SET OUTPUT TO TOGGLE ON MATCH	CH3
		TIM2->CCMR2 = (0x6 << 12);	//SET OUTPUT TO TOGGLE ON MATCH	CH4

		TIM2->CCMR1 |= (1 << 3);	// enable oc1 preload bit 3
		TIM2->CCMR2 |= (1 << 3);	// enable oc3 preload bit 3
		TIM2->CCMR2 |= (1 << 11);	// enable oc4 preload bit 3

		TIM2->EGR  |= (1 << 0);

		TIM2->CCER |= (1 << 0);		// enable capture/compare ch1 output
		TIM2->CCER |= (1 << 8);		// enable capture/compare ch3 output
		TIM2->CCER |= (1 << 12);		// enable capture/compare ch4 output

		TIM2->CNT = 0;			//CLEAR COUNTER
		TIM2->CR1 = 1;			//ENABLE TIM2


}




