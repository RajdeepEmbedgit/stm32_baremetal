// Adapted from the STM32F4-Discovery project

/***************************************************************
* File     : fm_stm32f4_led.c
* website  : www.firmcodes.com
* email_id : support@firmcodes.com
* IDE used : The files are Compiled in coocox ide and
			 tested on stm32f4 discovery board.
***************************************************************/
#include <fm_stm32f4_led.h>




/***************************************************************
* All LED on Stm32f4 discovery board
***************************************************************/
LED_struct LED[] = {
  //Name, PORT , PIN       , CLOCK            , Init
  {LED_1,GPIOB,GPIO_Pin_0,RCC_AHB1Periph_GPIOB,LED_OFF},	   // PB0 = Green LED on Discovery-Board
  {LED_2,GPIOB,GPIO_Pin_7,RCC_AHB1Periph_GPIOB,LED_OFF},   	   // PB7=Blue LED on Discovery-Board
  //{LED_3,GPIOB,GPIO_Pin_14,RCC_AHB1Periph_GPIOB,LED_OFF},    // PB14=Red LED on Discovery-Board

};

/***************************************************************
* Initialize all LED
***************************************************************/
void FM_Led_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  LED_NAME led_name;

  for(led_name=0;led_name<TOTAL_LED;led_name++) {

  // Clock Enable
    RCC_AHB1PeriphClockCmd(LED[led_name].LED_CLK, ENABLE);

    // Config as digital output
    GPIO_InitStructure.GPIO_Pin = LED[led_name].LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED[led_name].LED_PORT, &GPIO_InitStructure);

  }
}


/***************************************************************
* LED off
***************************************************************/
void FM_Led_Off(LED_NAME led_name)
{
  LED[led_name].LED_PORT->BSRRH = LED[led_name].LED_PIN;
}

/***************************************************************
* LED on
***************************************************************/
void FM_Led_On(LED_NAME led_name)
{
  LED[led_name].LED_PORT->BSRRL = LED[led_name].LED_PIN;
}

/***************************************************************
* LED toggle
***************************************************************/
void FM_Led_Toggle(LED_NAME led_name)
{

  LED[led_name].LED_PORT->ODR ^= LED[led_name].LED_PIN;

}


