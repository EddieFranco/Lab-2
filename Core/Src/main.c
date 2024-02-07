/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* USER CODE END 0 */
void EXTI0_1_IRQHandler()
		{
		 volatile uint32_t counter = 0;
// Toggle both pin output states within the endless loop.
		GPIOC->ODR ^= (GPIO_ODR_9 | GPIO_ODR_8); // Toggle PC8 and PC9
		//HAL_Delay(200); // Delay 200ms

		while(	counter <=1500000){
			counter++;
		}
		
		GPIOC->ODR ^= (GPIO_ODR_6 | GPIO_ODR_7);
		
		counter =0; 
		
		EXTI->PR |= EXTI_PR_PR0_Msk ;
	
		}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  SystemClock_Config();

	

 __HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
// Set up a configuration struct to pass to the initialization function
GPIO_InitTypeDef initStr = {GPIO_PIN_6|GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9,
GPIO_MODE_OUTPUT_PP,
GPIO_SPEED_FREQ_LOW,
GPIO_NOPULL};

HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins 

HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Start PC9 high



//Configure the button pin (PA0) to input-mode at low-speed, with the internal pull-down
//resistor enabled.
RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//Set to input mode in the MODER register.
GPIOA->MODER &= ~ (GPIO_MODER_MODER0_Msk);
//Set  to low speed in the OSPEEDR register
GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEEDR0_Msk);
//Enable the pull-down resistor in the PUPDR register
GPIOA->OSPEEDR &= ~(GPIO_PUPDR_PUPDR0_Msk);


// Enable/unmask interrupt generation on EXTI input line 0 (EXTI0).
EXTI->IMR |= EXTI_IMR_MR0_Msk;
// Configure the EXTI input line 0 to have a rising-edge trigger.
EXTI->RTSR |= EXTI_RTSR_TR0_Msk ;

//Use the RCC to enable the peripheral clock to the SYSCFG peripheral.
RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN_Msk ;
// Configure the multiplexer to route PA0 to the EXTI input line 0 (EXTI0).
SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

//Enable the selected EXTI interrupt by passing its defined name
	NVIC_EnableIRQ(5);
// Set the priority for the interrupt to 1 (high-priority)

	NVIC_SetPriority(SysTick_IRQn, 2);
	NVIC_SetPriority(EXTI0_1_IRQn, 3);
	
//changes the SysTick interrupt priority to 2	
	
	//NVIC_SetPriority(EXTI0_1_IRQn, 2);
	//NVIC_SetPriority(EXTI0_1_IRQn, 3);

while (1) {
HAL_Delay(400); // Delay 200ms
// Toggle the output state of both PC8 and PC9
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6 );

} 
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
