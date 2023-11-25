/*
 * RTC.h
 *
 *  Created on: Jul 20, 2022
 *      Author: rajdeep
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "stm32f4xx.h"                  // Device header


void RTC_Init(void);
void RTC_get_time (void);
void RTC_get_date(void);


//char time[10];



#endif /* INC_RTC_H_ */
