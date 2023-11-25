/*
 * delay.h
 *
 *  Created on: Jul 20, 2022
 *      Author: rajdeep
 */

#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include "stm32f4xx.h"                  // Device header
#include <stdint.h>

void TIM6_Config (void);

void Delay_us (uint16_t us);

void Delay_ms (uint16_t ms);


#endif /* INC_DELAY_H_ */
