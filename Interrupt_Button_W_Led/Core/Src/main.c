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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void EXTI1_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t flg= 0;
volatile int portPG1counter;
volatile int portPG0counter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
//  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  TIM6Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  //************************	Interrupt	***************************//
  	  __disable_irq();

  // setup LEDs
    RCC->AHB1ENR |= (1<<1);		// FOR PORT B
    RCC->AHB1ENR |= (1<<2);		// FOR PORT C
    RCC->AHB1ENR |= (1<<6);		// FOR PORT G
    					/*****	   LED		*****/
 	  GPIOB->MODER |= (1<<0);		//	General purpose output mode PIN 0
 	  GPIOB->MODER |= (1<<14);		//	General purpose output mode PIN 7
 	  GPIOB->MODER |= (1<<28);		//	General purpose output mode PIN 14
 	  	  	  	  	  /*****	  BUTTONS		*****/
 	  GPIOC->MODER |= (1<<26);		//	General purpose output mode PIN 13 ((USER BUTTON))		PC13

 	  	  	  	  	  	  	  /*******	 FOR PG1	*******/
 	  GPIOG->MODER |= (1<<1);		//	General purpose output mode PIN 1   ((EXT BUTTON))		PG1
 	  GPIOG->PUPDR	&=	~(GPIO_PUPDR_PUPDR1);

 	  	  	  	  	  	  	  	  	  	  	  	  /*****	 INTERRUPT		*****/
 	  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;



 	  	  	  	  	  	  	  /*****	 INTERRUPT FOR PG1 BUTTON		*****/
 	  SYSCFG->EXTICR[0] &= ~(0xF << 4);
 	  	  	  	  	  	  	  	  	  /*******	 FOR PG1	*******/
 	  SYSCFG->EXTICR[0] |= (0<<7);		//external interrupt configuration register
 	  SYSCFG->EXTICR[0] |= (1<<6);
 	  SYSCFG->EXTICR[0] |= (1<<5);
 	  SYSCFG->EXTICR[0] |= (0<<4);

 	  EXTI->IMR |= (1<<1);		//Interrupt mask register
 	  EXTI->FTSR |=	(1<<1);		//Falling trigger selection register

 	  NVIC_EnableIRQ(EXTI1_IRQn);	//Interrupt Handler
 	  NVIC_SetPriority(EXTI1_IRQn, 0);

 	  __enable_irq();


     	 //**********************************************************//


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  GPIOB->ODR ^= (1<<0);
	  HAL_Delay(200);

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/*************************************************
* interrupt handler	PG1 EXT BUTTON
*************************************************/
void EXTI1_IRQHandler(void)
{
	if(flg == 0) //on
	{
		if(GPIOG->IDR == 0xf7bc)
			{
				GPIOB->ODR	|=	(1<<7);
			}
		flg = 1;
	}

	else if(flg == 1)
	{
		if(GPIOG->IDR == 0xf7bc)
			{
				GPIOB->ODR &= ~(1<<7);
			}

		flg = 0;
	}

EXTI->PR |=	EXTI_PR_PR1;	//This bit is set when the selected edge event arrives on the external interrupt line.


									/*		PG1 EXT USER BUTTON COUNTER		*/
//if(EXTI->PR &	EXTI_PR_PR1)
//{
//	portPG1counter++;
//	EXTI->PR |=	EXTI_PR_PR1;		//PG1
//}
//
//if(EXTI->PR &	EXTI_PR_PR0)
//{
//	portPG0counter++;
//	EXTI->PR |=	EXTI_PR_PR0;		//PG0
//}




}

/*************************************************
* interrupt handler	PC13 USER BUTTON
*************************************************/
//void EXTI15_10_IRQHandler(void)
//{
//										/*		PC13 EXT USER BUTTON COUNTER		*/
//if(EXTI->PR &	EXTI_PR_PR13)
//{
//	portPC13counter++;
//	EXTI->PR |=	EXTI_PR_PR13;		//PG1
//}
//
//}



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
