/*
 * gpio.c
 *
 *  Created on: Jun 22, 2022
 *      Author: Admin
 */

#include "stm32f4xx.h"

void GPIO_Config (void)
{


	  RCC->AHB1ENR |= (1<<1);		// FOR PORT B

	  GPIOB->MODER |= (1<<0);		//	General purpose output mode PIN 0
	  GPIOB->MODER |= (1<<14);		//	General purpose output mode PIN 7
	  GPIOB->MODER |= (1<<28);		//	General purpose output mode PIN 14

	  GPIOB->OTYPER = 0;

	  GPIOB->OSPEEDR = 0;


}
