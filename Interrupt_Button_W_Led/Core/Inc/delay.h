/*
 * delay.h
 *
 *  Created on: 09-Jul-2022
 *      Author: rajdeep
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

void TIM6Config (void);
void Delay_us (uint32_t us);
void Delay_ms (uint32_t ms);

#endif /* INC_DELAY_H_ */
