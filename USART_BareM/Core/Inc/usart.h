/*
 * usart.h
 *
 *  Created on: Jul 13, 2022
 *      Author: rajdeep
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"


void uart3_pin_conf();

void UART3_SendChar(USART_TypeDef* USARTx, uint8_t ch);
uint32_t HART_USART_IsActiveFlag_TXE(USART_TypeDef *USARTx);
void HART_USART_TransmitData8(USART_TypeDef *USARTx, uint8_t Value);
uint32_t HART_USART_IsActiveFlag_TC(USART_TypeDef *USARTx);

void UART3_SendChar_RCV (char c);
uint8_t UART3_GetChar (void);

void USART_write(int ch);
void UART_Write_String(char *p);

#endif /* INC_USART_H_ */
