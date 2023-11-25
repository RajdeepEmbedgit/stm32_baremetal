/*
 * pwm_hart.h
 *
 *  Created on: Aug 16, 2022
 *      Author: rajdeep
 */

#ifndef INC_PWM_HART_H_
#define INC_PWM_HART_H_

#include "stm32f4xx.h"

void PWM_input_Init(void);

void time_output_CC_init (void);

void PWM_input_captureInit(void);

void TIM2_IRQHandler(void);

void start_Tim3 (void);

void Frequency_Measure ();


extern uint16_t IC1Value;
extern uint16_t IC2Value;
extern float DutyCycle;
extern float Frequency;


#endif /* INC_PWM_HART_H_ */
