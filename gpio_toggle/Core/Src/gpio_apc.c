/*
 * gpio_apc.c
 *
 *  Created on: Nov 8, 2022
 *      Author: rajdeep
 */

#include "stm32f429xx.h"

void gpio_init(void)
{
	 // setup LEDs
	     RCC->AHB1ENR |= (1<<1);		// FOR PORT B
	     RCC->AHB1ENR |= (1<<2);		// FOR PORT C
	     RCC->AHB1ENR |= (1<<6);		// FOR PORT G
	     					      /*****	   LED	  *****/
	  	  GPIOB->MODER |= (1<<0);		//	General purpose output mode PIN 0
	  	  GPIOB->MODER |= (1<<14);		//	General purpose output mode PIN 7
	  	  GPIOB->MODER |= (1<<28);		//	General purpose output mode PIN 14
}




