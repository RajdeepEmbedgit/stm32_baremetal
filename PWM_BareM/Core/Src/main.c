/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pwm.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void set_rgb (uint8_t red, uint8_t green, uint8_t blue);

void TIM4_IRQHandler(void);

#define PWMPERIOD 2000
#define SAMPLE 2000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void set_rgb (uint8_t red, uint8_t green, uint8_t blue)
{
 TIM2->CCR1 = red;
 TIM2->CCR2 = green;
 TIM2->CCR3 = blue;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
//  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  GPIO_Config();
//  HART_Tim_init();
  pwm_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	  unsigned int rgbColour[3];

	 	  // Start  with 	White.
	 	 /*  rgbColour[0] = 999;
	 	   rgbColour[1] = 999;
	 	   rgbColour[2] = 999;*/

	 	   for (int decColour = 0; decColour < 3; decColour += 1) {
	 	     int incColour = decColour == 2 ? 0 : decColour + 1;
	 	     // cross-fade the two colours.
	 	     for(int i = 0; i < 255; i += 1) {

	 	       rgbColour[decColour] -= 1;
	 	       rgbColour[incColour] += 1;

	 	       set_rgb(rgbColour[0], 0, rgbColour[2]);

	 	       set_rgb(0, rgbColour[1], rgbColour[2]);

	 	      set_rgb(rgbColour[0], rgbColour[1], 0);

	 	       set_rgb(rgbColour[0], rgbColour[1], rgbColour[2]);	//white colour

	 	       HAL_Delay (1);
	 	     }
	 	   }


	  int x;
	 	  for(x=0; x<1000; x++)
	 	  {
	 		  TIM2->CCR1 = x;
	 		  HAL_Delay(1);
	 	  }

	 	  for(x=0; x<1000; x++)
	 	  {
	 		  TIM2->CCR3 = x;
	 		  HAL_Delay(1);
	 	  }

	 	  for(x=0; x<1000; x++)
	 	  {
	 		  TIM2->CCR4 = x;
	 		  HAL_Delay(1);
	 	  }

	 	  	  for(x=1000; x>0; x--)
	 	  	  {
	 	  		  TIM2->CCR1 = x;
	 	  		  HAL_Delay(5);
	 	  	  }

	 	  	   for(x=1000; x>0; x--)
	 	  	  {
	 	  		  TIM2->CCR3 = x;
	 	  		  HAL_Delay(5);
	 	  	  }

	 	  	   for(x=1000; x>0; x--)
	 	  	  {
	 	  		  TIM2->CCR4 = x;
	 	  		  HAL_Delay(5);
	 	  	  }



  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*

void TIM4_IRQHandler(void)
{
    static uint32_t t = 0;
    static uint16_t duty = 0;

    // clear interrupt status
    if (TIM1->DIER & 0x01) {
        if (TIM4->SR & TIM_SR_UIF) {
        	TIM4->SR &=~ TIM_SR_UIF;
        }
    }

    duty =(uint16_t)(PWMPERIOD/2.0 * (sin(2*M_PI*(double)t/(SAMPLE)) + 1.0));
    ++t;
    if (t == SAMPLE) t = 0;
    // set new duty cycle
    TIM4->CCR1 = duty;
}

*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
