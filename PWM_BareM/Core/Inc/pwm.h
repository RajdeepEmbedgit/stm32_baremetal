/*
 * pwm.h
 *
 *  Created on: Jul 7, 2022
 *      Author: rajdeep
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "stm32f4xx.h"

void pwm_init(void);
void HART_Tim_init (void);
void start_Tim (void);



#endif /* INC_PWM_H_ */
