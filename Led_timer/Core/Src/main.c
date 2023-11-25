/*
**
**                           Main.c
**
**
**********************************************************************/
/*
   AUTHOR:          Roan Fourie
   DATE CREATED:    2017-Week-38
   DATE CHANGED:    2017-Week-38

**********************************************************************/
#include <fm_stm32f4_delay.h>
#include <fm_stm32f4_led.h>
#include <stm32f4xx_conf.h>
#include "stm32f4xx.h"
#include "tim.h"

#include "../SPL/inc/stm32f4xx_gpio.h"
#include "../SPL/inc/stm32f4xx_rcc.h"

void button_pressed(void);
uint16_t time(void);
void TIM2_IRQHandler(void);

uint16_t x;

// SystemCoreClock = ((INPUT_CLOCK(HSE_OR_HSI_IN_HZ)/PLL_M)*PLL_N)/PLL_P

static void button_setup(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

	// Clock Enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    // Config as digital output
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    	// Config as digital output
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOG, &GPIO_InitStructure);
}

int main(void)
{
    // initialize the system frequency
    SystemInit();
    // Delay initialize
    delay_init();
    // all LED initialize
    FM_Led_Init();
    // Initialize the button
    button_setup();
    tim2_1hz_init();

  while(1)
  {
	 /* start_Tim2();
	  GPIOB->ODR ^= (1 << 0);
*/

	  button_pressed();
  }
}
/*
uint16_t time(void)
{
	FM_Led_Toggle(LED_1);
	delay_ms(1000);
}*/

void button_pressed(void)
{
	if (GPIO_ReadInputDataBit(GPIOG, GPIO_Pin_1))		//External Button	//(GPIOA, GPIO_Pin_13)) User Button
	     {
	   	FM_Led_Toggle(LED_1);
	       FM_Led_Off(LED_2);
	     } else {
	       FM_Led_Toggle(LED_2);
	       FM_Led_Off(LED_1);
	     }
	       // Delay for half second
	      delay_ms(1000);
}
