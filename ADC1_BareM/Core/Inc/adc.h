/*
 * adc.h
 *
 *  Created on: 06-Jul-2022
 *      Author: rajdeep
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "stm32f4xx.h"

void ADC_ch1_init(void);
void ADC_conversion(void);
uint32_t ADC_read(void);
uint32_t ADC3_read(void);


void ADC_Init (void);
void ADC_Enable (void);
void ADC_Start (int channel);
void ADC_WaitForConv (void);
uint32_t ADC_GetVal (void);
void ADC_Disable (void);


#endif /* INC_ADC_H_ */
