/*
 * usart.c
 *
 *  Created on: Jul 13, 2022
 *      Author: rajdeep
 */

#include "usart.h"

void uart3_pin_conf()
{
						//***************************GPIO****************************//

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;		//Enable GPIOD Clock  PD8/PD9

	GPIOD->MODER |= (2<<16);			//Bits (21:20) = 1:0 --> Alternate function for Pin PD8
	GPIOD->MODER |= (2<<18);			//Bits (23:22) = 1:0 --> Alternate function for Pin PD9

//	GPIOD->OSPEEDR |= (3<<16) | (3<<18);   //Bits (17:16)=(1:1) & Bits(19:18)=(1:1) -->> High speed for PIN PD8 & PD9

	GPIOD->AFR[1] |= (7<<0);	// 0:1:1:1 --> AF7 Alternate function for USART3 at pin PD8
	GPIOD->AFR[1] |= (7<<4);	// 0:1:1:1 --> AF7 Alternate function for USART3 at pin PD9

							//****************** USART **********************//

	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;		//Enable UART3 Clock

	//Program for 8bit	(M Word length)
	USART3->CR1 &= ~(1<<12);

	//Enable the USART
	USART3->CR1 |= (1<<13);	//UE = 1  Enable USART


//	USART3->BRR |= 		(7<<0) | (24<<4);		//Program for BaudRate 115200,PCLK 45Mhz
	USART3->BRR |= 		(15<<0) | (292<<4);		//9600 baudrate for 45Mhz

	//Enable the Tx/Rx
	USART3->CR1 |= (1<<2);	//Recevier Enable	PD8
	USART3->CR1 |= (1<<3);	//Transmiter Enable	PD9


}


void UART3_SendChar_RCV (char c)
{
	/*********** STEPS FOLLOWED *************

	1. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
		 for each data to be transmitted in case of single buffer.
	2. After writing the last data into the USART_DR register, wait until TC=1. This indicates
		 that the transmission of the last frame is complete. This is required for instance when
		 the USART is disabled or enters the Halt mode to avoid corrupting the last transmission.

	****************************************/

	USART3->DR = c;   // LOad the Data
	while (!(USART3->SR & (1<<6)));  // Wait for TC to SET.. This indicates that the data has been transmitted
}


uint8_t UART3_GetChar (void)
{
		/*********** STEPS FOLLOWED *************

	1. Wait for the RXNE bit to set. It indicates that the data has been received and can be read.
	2. Read the data from USART_DR  Register. This also clears the RXNE bit

	****************************************/
	uint8_t Temp;

	while (!(USART3->SR & (1<<5)));  // Wait for RXNE to SET.. This indicates that the data has been Received
	Temp = USART3->DR;  // Read the data.
	return Temp;
}

					/////////////////////////	STRING SEND  ////////////////////////////

void USART_write(int ch){
while(!(USART3->SR & 0x0080)){
}
USART3->DR=(ch);

}


void UART_Write_String(char *p)
	{
	while(*p!='\0')
{
USART_write(*p);
p++;
}
}
							//////////////////////	END  /////////////////////////




/*
uint32_t HART_USART_IsActiveFlag_TXE(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_TXE) == (USART_SR_TXE));
}

void HART_USART_TransmitData8(USART_TypeDef *USARTx, uint8_t Value)
{
  USARTx->DR = Value;
}

uint32_t HART_USART_IsActiveFlag_TC(USART_TypeDef *USARTx)
{
  return (READ_BIT(USARTx->SR, USART_SR_TC) == (USART_SR_TC));
}

*/
