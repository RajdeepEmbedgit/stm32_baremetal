/*
 * i2c.c
 *
 *  Created on: Sep 30, 2022
 *      Author: rajdeep
 */


#include "stm32f429xx.h"

			      ////////////////////////// STEPS FOLLOWED  ////////////////////////////////////
						/*  1. Enable the I2C CLOCK and GPIO CLOCK
							2. Configure the I2C PINs for ALternate Functions
								a) Select Alternate Function in MODER Register
								b) Select Open Drain Output
								c) Select High SPEED for the PINs
								d) Select Pull-up for both the Pins
								e) Configure the Alternate Function in AFR Register
						    3. Reset the I2C
							4. Program the peripheral input clock in I2C_CR2 Register in
							   order to generate correct timings
							5. Configure the clock control registers
							6. Configure the rise time register
							7. Program the I2C_CR1 register to enable the peripheral*/
		         /////////////////////////////////////////////////////////////////////////////////

void I2C2_Hart(void)
{
	RCC->APB1ENR |= (1 << 22);			//ENABLE I2C2 CLOCK
	RCC->AHB1ENR |= (1 << 5);			//ENABLE GPIOF CLOCK
	GPIOF->MODER |= ()

}
