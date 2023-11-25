/*
 * input_cap.c
 *
 *  Created on: Aug 24, 2022
 *      Author: rajdeep
 */

#include "stm32f4xx.h"
#include "input_cap.h"


#define TIMCLOCK   90000000
#define PRESCALAR  90


	 uint8_t input_start = 0;


	uint16_t	Last_value = 0;
	uint16_t	Current_value = 0;
	uint16_t	period = 0;
	uint32_t    usWidth = 0;

	int i = 0;

	int Is_First_Captured = 0;
	float refClock;

	float frequency = 0.0;

	float b_sinwavefreq;
	float b_pwmprdupdate;

//	char freq[13] = {0};


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
		TIM3->PSC = 90-1;			//16000;
		TIM3->ARR = 30000-1;

		//TIM3->SMCR = 0;
		TIM3->SMCR |= (1 << 7);		//MSM ACTIVE MASTER/SLAVE		//optional
		TIM3->SMCR |= (5 << 4);		//TS=101 trigger selection
		TIM3->SMCR |= (4 << 0); 	//SMS=100 reset mode
			////////////////////	TIM3_CH3	///////////////////

		TIM3->CCMR2 |= (1 << 0);	//CC3S = 01 SELECT INPUT
		TIM3->CCMR2 |= (3 << 4);	//IC3F = 0011 Config input filter
		TIM3->CCER  |= (0 << 9);	//CC3P = 0 Select valid conversion edge Rising edge valid
		TIM3->CCMR2 |= (0 << 2);   //IC3PS = 00 Configure input freq de=ivision
		TIM3->CCER  |= (1 << 8);	//CC3E = 1 allows to capture the value of the counter into the capture register

			///////////////////////////////////////////////////////

		//TIM3->SMCR = 0;
//		TIM3->SMCR |= (1 << 7);		//MSM ACTIVE MASTER/SLAVE		//optional
		TIM3->SMCR |= (6 << 4);		//TS=110 trigger selection
		TIM3->SMCR |= (4 << 0); 	//SMS=100 reset mode


			///////////////////		TIM3_CH4	///////////////////
		TIM3->CCMR2 |= (2 << 8);		//CC4S = 10 Select input
		TIM3->CCER  |= (1 << 13);		//CC4P = 1 Select the falling edge of the alternating conversion edge to be valid
		TIM3->CCER	|= (1 << 12);		//CC4E = 1  allows to capture the value of the counter into the capture register

			///////////////////////////////////////////////////////

		TIM3->DIER  |= TIM_DIER_CC3IE; //Allow update capture interrupt


		NVIC_EnableIRQ(TIM3_IRQn);
//		NVIC_SetPriority(TIM3_IRQn, 0);



		TIM3->CNT = 0;


		//////////////////////////////////////////////////////////



		//////////////////////////////////////////////////////////

		TIM3->CR1 = 1;

		//////////////////////////////////////////////////////////



//		time_elapsed = TIM3->CNT;
		//////////////////////////////////////////////////////////



		TIM3->EGR |= TIM_EGR_UG;

//	__enable_irq();
 }


void TIM3_IRQHandler(void)
{

	if (TIM3->DIER |= TIM_DIER_CC3IE)
	{
	    	 if (TIM3->SR & TIM_SR_CC3IF)
	    		  {

			//////////////////////////////////////////////////////////////////////////////////////////////////

	    		 	 	 if (Is_First_Captured == 0)		// if the first value is not captured
							  	  {
							  		  	  Current_value = TIM3->CCR3;		//first value capture
							  		  	  Is_First_Captured = 1;

							  	  }
							  	  else		// if the first is already captured
							  	  {

							  		  	  Last_value = TIM3->CCR3;	//Read last value capture

							  		  	  if(Current_value != 0)
							  				{
							  					period = (Last_value - Current_value);

							  				}

							  			refClock = (TIMCLOCK / TIM3->PSC) ;

							  			frequency = (refClock/period);

							  			///////////////////////////////////////////////////


							  			///////////////////////////////////////////////////

							  		  	  Is_First_Captured = 0;

							  	  }

			//////////////////////////////////////////////////////////////////////////////////////////////////

	    		}

	   }

}


float Input_capture(void)
{
	 //////////////////////// WORKING ////////////////////////////////
	  if(input_start  == 1)
	  {

//		  while(!(USART3->SR & (1<<7)));
//		  			USART3->DR = 'B';

//				while(!(TIM3->SR & 8)){}

				/*  if (Is_First_Captured == 0)		// if the first value is not captured
				  	  {
				  		  	  Current_value = TIM3->CCR3;		//first value capture
				  		  	  Is_First_Captured = 1;

				  	  }
				  	  else		// if the first is already captured
				  	  {

				  		  	  Last_value = TIM3->CCR3;	//Read last value capture

				  		  	  if(Current_value != 0)
				  				{
				  					period = (Last_value - Current_value);

				  				}

				  			refClock = (TIMCLOCK / TIM3->PSC) ;

				  			frequency = (refClock/period);

				  		  	  Is_First_Captured = 0;

				  	  }*/

				  input_start = 0;

				  	  i++;

			return frequency;
	  }

//	  sprintf(freq,"%d\r\n",frequency);
//
//	  UART_Write_String(freq);

//	  HAL_Delay(500);


				//////////////////////////////////////////////////////////////////////////

}
