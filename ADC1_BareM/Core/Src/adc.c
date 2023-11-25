/*
 * adc.c
 *
 *  Created on: 06-Jul-2022
 *      Author: rajdeep
 */

#include "main.h"

void ADC_ch1_init(void)
{
									/* ADC1 PIN PA3 */

		RCC->APB2ENR |= (1<<8);  	// Enable clock source for ADC1		<<---- 	ok
		RCC->AHB1ENR |= (1<<0); 	// Enable clock source for PORT A	<<----	ok

									/* ADC3 PIN PF3 */

		RCC->APB2ENR |= (1<<10);  	// Enable clock source for ADC3		<<---- 	ok
		RCC->AHB1ENR |= (1<<5); 	// Enable clock source for PORT F	<<----	ok
				/*********************	ADC 1 ************************/

		ADC->CCR |= (3<<16);		// PCLK2 Divide by 8

		ADC1->CR1 = (1<<8);		//  SCAN Mode Enable
		ADC1->CR1 &= ~(1 << 24);	//  set 12 bit ADC					<<----
		ADC1->CR1 &= ~(1 << 25);	//  set 12 bit ADC					<<----

		//4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
		ADC1->CR2 = (1<<1);     	// enable continuous conversion mode
		ADC1->CR2 |= (1<<10);    	// EOC after each conversion
		ADC1->CR2 &= ~(1<<11);   	// Data Alignment RIGHT

		ADC1->SMPR2 &= ~(7<<6);  	// Sampling time of 3 cycles for channel 3

		ADC1->SQR1 |= (0<<20);   	// SQR1_L =1 for 1 conversions
		ADC1->SQR1 |= (0<<21);   	// SQR1_L =1 for 1 conversions
		ADC1->SQR1 |= (0<<22);   	// SQR1_L =1 for 1 conversions
		ADC1->SQR1 |= (0<<23);   	// SQR1_L =1 for 1 conversions

		ADC1->SQR3 |= (3<<0);  // SEQ1 for Channel 3

				/*****************************************************************/

					/*********************	ADC 3 ************************/

				ADC->CCR |= (3<<16);		// PCLK2 Divide by 8

				ADC3->CR1 = (1<<8);		//  SCAN Mode Enable
				ADC3->CR1 &= ~(1 << 24);	//  set 12 bit ADC					<<----
				ADC3->CR1 &= ~(1 << 25);	//  set 12 bit ADC					<<----

				//4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
				ADC3->CR2 = (1<<1);     	// enable continuous conversion mode
				ADC3->CR2 |= (1<<10);    	// EOC after each conversion
				ADC3->CR2 &= ~(1<<11);   	// Data Alignment RIGHT

				ADC3->SMPR2 &= ~(7<<24);  	// Sampling time of 3 cycles for channel 9

				ADC3->SQR1 |= (0<<20);   	// SQR1_L =1 for 1 conversions
				ADC3->SQR1 |= (0<<21);   	// SQR1_L =1 for 1 conversions
				ADC3->SQR1 |= (0<<22);   	// SQR1_L =1 for 1 conversions
				ADC3->SQR1 |= (0<<23);   	// SQR1_L =1 for 1 conversions

				ADC3->SQR3 |= (9<<0);  // SEQ1 for Channel 9

			/*****************************************************************/

		GPIOA->MODER |= (3 << 6);	//Analog mode for PA3				<<----	ok
		GPIOF->MODER |= (3 << 6);	//Analog mode for PF3				<<----	ok

		ADC1->CR2 |= ADC_CR2_ADON;	//	ADC1 On							<<----	ok
		ADC3->CR2 |= ADC_CR2_ADON;	//	ADC3 On							<<----	ok

}


void ADC_conversion(void)
{
	ADC1->SR = 0;
	ADC1->CR2 |= (1<<30); // Start  ADC conversion

	ADC3->SR = 0;
	ADC3->CR2 |= (1<<30); // Start  ADC conversion
}

uint32_t ADC_read(void)
{
	while(!(ADC1->SR & (1<<1))) {} // wait for conversion to be complete

		return (ADC1-> DR);			//Read Converted result

}

uint32_t ADC3_read(void)
{
	while(!(ADC3->SR & (1<<1))) {} // wait for conversion to be complete

		return (ADC3-> DR);			//Read Converted result

}




////////////////////// Two channel adc /////////////////////////////////
/*
void ADC_Init (void)
{


//1. Enable ADC and GPIO clock
	RCC->APB2ENR |= (1<<8);  // enable ADC1 clock		//
	RCC->AHB1ENR |= (1<<0);  // enable GPIOA clock		//
//	RCC->AHB1ENR |= (1<<5);  // enable GPIOF clock		//

//2. Set the prescalar in the Common Control Register (CCR)
	ADC->CCR |= 1<<16;  		 // PCLK2 divide by 4

//3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)
	ADC1->CR1 |= (1U<<8);    // SCAN mode enabled
	ADC1->CR1 |= (0<<24);		//resolution
	ADC1->CR1 |= (0<<25);		//resolution


//4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	ADC1->CR2 &= ~(1<<1);		//|= (1<<1);     // enable continuous conversion mode
	ADC1->CR2 &= ~(1<<11);   // Data Alignment RIGHT

	ADC1->CR2 |= (1<<10);    // EOC after each conversion

//5. Set the Sampling Time for the channels

	ADC1->SMPR2 |= (7 << 0);
//	ADC1->SMPR2 &= ~((1<<3) | (1<<12));  // Sampling time of 3 cycles for channel 1 and channel 4

//6. Set the Regular channel sequence length in ADC_SQR1
	ADC1->SQR1 |= (1<<20);   // SQR1_L =1 for 2 conversions

//7. Set the Respective GPIO PINs in the Analog Mode
	GPIOA->MODER |= (3U <<  6);  // analog mode for PA 3 (chennel 1)
}


void ADC_Enable (void)
{

	ADC1->CR2 |= 1<<0;   // ADON =1 enable ADC1

	uint32_t delay = 10000;
	while (delay--);
}


void ADC_Start (int channel)
{

	ADC1->SQR3 = 0;
	ADC1->SQR3 |= (channel<<0);    // conversion in regular sequence

	ADC1->SR = 0;        // clear the status register

	ADC1->CR2 |= (1<<30);  // start the conversion
}


void ADC_WaitForConv (void)
{
	while (!(ADC1->SR & (1<<1)));  // wait for EOC flag to set
}

uint32_t ADC_GetVal (void)
{
	return ADC1->DR;  // Read the Data Register
}

void ADC_Disable (void)
{
	ADC1->CR2 &= ~(1<<0);  // Disable ADC
}

*/

/////////////////////// end /////////////////////////


