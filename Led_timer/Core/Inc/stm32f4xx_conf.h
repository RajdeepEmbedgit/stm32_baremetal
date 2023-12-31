/**
*****************************************************************************
**
**  File        : stm32f4xx_conf.h
**
**  Abstract    : STM32F4xx library configuration file
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed �as is,� without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_CONF_H
#define __STM32F4xx_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment the line below to enable peripheral header file inclusion */
#include "../SPL/inc/misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */
#include "../SPL/inc/stm32f4xx_adc.h"
#include "../SPL/inc/stm32f4xx_can.h"
#include "../SPL/inc/stm32f4xx_crc.h"
#include "../SPL/inc/stm32f4xx_cryp.h"
#include "../SPL/inc/stm32f4xx_dac.h"
#include "../SPL/inc/stm32f4xx_dbgmcu.h"
#include "../SPL/inc/stm32f4xx_dcmi.h"
#include "../SPL/inc/stm32f4xx_dma.h"
#include "../SPL/inc/stm32f4xx_exti.h"
#include "../SPL/inc/stm32f4xx_flash.h"
#include "../SPL/inc/stm32f4xx_fsmc.h"
#include "../SPL/inc/stm32f4xx_gpio.h"
#include "../SPL/inc/stm32f4xx_hash.h"
#include "../SPL/inc/stm32f4xx_i2c.h"
#include "../SPL/inc/stm32f4xx_iwdg.h"
#include "../SPL/inc/stm32f4xx_pwr.h"
#include "../SPL/inc/stm32f4xx_rcc.h"
#include "../SPL/inc/stm32f4xx_rng.h"
#include "../SPL/inc/stm32f4xx_rtc.h"
#include "../SPL/inc/stm32f4xx_sdio.h"
#include "../SPL/inc/stm32f4xx_spi.h"
#include "../SPL/inc/stm32f4xx_syscfg.h"
#include "../SPL/inc/stm32f4xx_tim.h"
#include "../SPL/inc/stm32f4xx_usart.h"
#include "../SPL/inc/stm32f4xx_wwdg.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* If an external clock source is used, then the value of the following define
   should be set to the value of the external clock source, else, if no external
   clock is used, keep this define commented */
/*#define I2S_EXTERNAL_CLOCK_VAL   12288000 */ /* Value of the external clock in Hz */


/* Uncomment the line below to expanse the "assert_param" macro in the
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *   which reports the name of the source file and the source
  *   line number of the call that failed.
  *   If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F45x_CONF_H */
