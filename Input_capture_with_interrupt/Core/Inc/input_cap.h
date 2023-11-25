/*
 * input_cap.h
 *
 *  Created on: Aug 24, 2022
 *      Author: rajdeep
 */

#ifndef INC_INPUT_CAP_H_
#define INC_INPUT_CAP_H_

void PWM_input_captureInit(void);
float Input_capture(void);

void PWM_input_capture_interrupt_Init(void);

extern  uint8_t input_start;
extern float frequency;

extern int i;

#endif /* INC_INPUT_CAP_H_ */
