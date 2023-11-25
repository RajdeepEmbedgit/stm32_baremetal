/*
 * pwm_hart.c
 *
 *  Created on: Aug 16, 2022
 *      Author: rajdeep
 */

#include "pwm_hart.h"


////////////////////////////////////////////

/*

	#define TIMCLOCK   90000000
	#define PRESCALAR  90

	uint16_t	Last_value = 0;
	uint16_t	Current_value = 0;
	uint16_t	period = 0;
	float frequency = 0;
	uint32_t usWidth = 0;
	float freq = 0.0;
	int Is_First_Captured = 0;
	float refClock;
*/

///////////////////////////////////////////

void time_output_CC_init (void)
{

	RCC->AHB1ENR |= (1<<0);
	GPIOA->MODER |= (1<<1);		//GPIO 10 :AF USED FOR PWM	SET PA0
	GPIOA->AFR[0] |= 0x00000001;	//SET PIN TO AF1 FOR TIM2_CH1

	//TIM2 CONFIGURE
	RCC->APB1ENR |= (1<<0);
	TIM2->PSC = 90-1;			//300-1;
	TIM2->ARR = 10000-1;			//300-1;
	TIM2->CCMR1 = 0x330;		//0x330;	//SET OUTPUT TO TOGGLE ON MATCH
	TIM2->CCR1 = 1;			//SET MATCH VALUE
	TIM2->CCER |= 1;		//ENABLE CH1 COMPARE MODE
	TIM2->CNT = 0;		//0;			//CLEAR COUNTER
	TIM2->CR1 = 1;			//ENABLE TIM2
}

//PWM output initialization
//arr: auto-reload value
//psc: clock prescaler

/*

void PWM_input_Init(void)
{
//	__disable_irq();

	RCC->AHB1ENR |= (1<<1);
	GPIOB->MODER |= (1<<1);		//GPIO 10 :AF USED FOR PWM	SET PB0
	GPIOB->AFR[0] |= 0x00000002;	//SET PIN TO AF2 FOR TIM3_CH3

	//TIM3 CONFIGURE
	RCC->APB1ENR |= (1<<1);
	TIM3->PSC = 90-1;			//16000;
	TIM3->ARR = 30000-1;
	TIM3->CCMR2 = 0x41;			//0100 0001 //SET CH3 TO CAPTURE AT EVRY EDGE
	TIM3->CCER = 0x0B00;
	TIM3->CNT = 0;		//MODIFY


//	TIM3->DIER  |= TIM_DIER_UIE; //Allow update capture interrupt

	TIM3->CR1 = 1;


//	NVIC_EnableIRQ(TIM2_IRQn);
//
//		__enable_irq();
}
*/


/*
void TIM2_IRQHandler(void)
{

//	IC1Value = TIM3->CCR3;	//Reading CCR3 can also clear the CC3IF flag
//	IC2Value = TIM3->CCR3;	//Reading CCR4 can also clear the CC4IF flag

	  	  if (Is_First_Captured == 0)		// if the first value is not captured
		  	  {
		  		  	  Current_value = TIM3->CCR3;		//first value capture
	//	  		  	  Last_value = TIM3->CCR4;	//Read last value capture
		  		  	  Is_First_Captured = 1;

	//	  		  	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		  	  }
		  	  else		// if the first is already captured
		  	  {

		  		  Last_value = TIM3->CCR3;	//Read last value capture

		  		  	  if(Current_value != 0)
		  				{
		  					period = (Last_value - Current_value);

	//	  					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		  				}

		  			refClock = (TIMCLOCK / TIM3->PSC) ;

		  			frequency = (refClock/period);

		  		  	  Is_First_Captured = 0;

		  	  }

	  return  frequency;


	TIM3->SR = 0;  	//Clear the interrupt flag

//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

}

*/

//////////////////	 FINAL INPUT CAPTURE	/////////////////////////

void PWM_input_captureInit(void)
{
//	__disable_irq();

		RCC->AHB1ENR |= (1<<1);
		GPIOB->MODER |= (1<<1);		//GPIO 10 :AF USED FOR PWM	SET PB0
		GPIOB->MODER |= (1<<3);		//GPIO 10 :AF USED FOR PWM	SET PB1
		GPIOB->AFR[0] |= 0x00000002;	//SET PIN TO AF2 FOR TIM3_CH3   PB0
		GPIOB->AFR[0] |= 0x00000020;	//SET PIN TO AF2 FOR TIM3_CH4	PB1

		//TIM3 CONFIGURE
		RCC->APB1ENR |= (1<<1);
		TIM3->PSC = 90-1;			//125;
		TIM3->ARR = 30000-1;		//25000

		//TIM3->SMCR = 0;
		TIM3->SMCR |= (1 << 7);		//MSM ACTIVE MASTER/SLAVE		//optional
		TIM3->SMCR |= (5 << 4);		//TS=101 trigger selection
		TIM3->SMCR |= (4 << 0); 	//SMS=100 reset mode
			////////////////////	TIM3_CH3	///////////////////
//		TIM3->CCMR2 = 0x41;			//0100 0001 //SET CH3 TO CAPTURE AT RISING EDGE	[[IC3F	----  CC3S]]
//
//		TIM3->CCER = 0x0B00;		//0000101100000000		CC3NP  RSV  CC3P  CC3E

		TIM3->CCMR2 |= (1 << 0);	//CC3S = 01 SELECT INPUT
		TIM3->CCMR2 |= (3 << 4);	//IC3F = 0011 Config input filter
		TIM3->CCER  |= (0 << 9);	//CC3P = 0 Select valid conversion edge Rising edge valid
		TIM3->CCMR2 |= (0 << 2);   //IC3PS = 00 Configure input freq de=ivision
		TIM3->CCER  |= (1 << 8);	//CC3E = 1 allows to capture the value of the counter into the capture register

			///////////////////////////////////////////////////////

			///////////////////		TIM3_CH4	///////////////////
//		TIM3->CCMR2 = 0X4200;			//0100 0010 //SET CH4 TO CAPTURE AT FALLING EDGE	[[IC4F	----  CC4S]]
//
//		TIM3->CCER =  0xB000;			//1011000000000000		CC4NP  RSV  CC4P  CC4E

		TIM3->CCMR2 |= (2 << 8);		//CC4S = 10 Select input
		TIM3->CCER  |= (1 << 13);		//CC4P = 1 Select the falling edge of the alternating conversion edge to be valid
		TIM3->CCER	|= (1 << 12);		//CC4E = 1  allows to capture the value of the counter into the capture register

			///////////////////////////////////////////////////////

//		TIM3->DIER  |= TIM_DIER_UIE; //Allow update capture interrupt

		TIM3->CR1 = 1;


//		NVIC_EnableIRQ(TIM2_IRQn);

//	__enable_irq();
 }

/////////////////////////////////////////////////////////////////////////////////////


/*
//INTERRUPT TIMER

void start_Tim3 (void)
{
	while(!(TIM3->SR & TIM_SR_UIF)){}

	TIM3->SR &=~ TIM_SR_UIF;

	IC1Value = TIM3->CCR3;	//Reading CCR3 can also clear the CC3IF flag
//	IC2Value = TIM3->CCR4;	//Reading CCR4 can also clear the CC4IF flag
}



////////////////////////////////// frequency ////////////////////////////////////

void Frequency_Measure()
{
	RCC->AHB1ENR |= (1<<0);
	GPIOA->MODER |= (1<<13);		//GPIO 10 :AF USED FOR PWM	SET PA6
	GPIOA->AFR[0] |= (1<<25);		//SET PIN TO AF2 FOR TIM3_CH3   PA6

	//TIM3 CONFIGURE
	RCC->APB1ENR |= (1<<1);
	TIM3->PSC = 16000-1;
	TIM3->CCMR1  |= (1<<0);
	TIM3->CCMR1 |= (1<<6);
	TIM3->CCER = (11<<0);			//DDDDDDD
	TIM3->CR1 = (1<<0);
}

*/
