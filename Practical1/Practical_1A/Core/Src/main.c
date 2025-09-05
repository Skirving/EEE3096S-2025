/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdint.h>
#include "stm32f0xx.h"
// using this library
//#include <stdlib.h> ASK TUTOR if can include this
#include <stdlib.h>

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
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
// TODO: Define input variables

// Mode tracking ADDED
//uint8_t current_mode = 0;
uint8_t led_position = 0;
int8_t direction = 1;

uint8_t sparkle_state = 0;         // 0 = generate pattern, 1 = start turning off
uint8_t sparkle_pattern = 0;
uint8_t sparkle_index = 0;
uint8_t sparkle_counter = 0;
uint32_t sparkle_timestamp = 0;
uint32_t sparkle_delay = 0;
uint32_t sparkle_hold_end_time = 0;  // Time when the hold period ends
uint8_t sparkle_off_delay = 0;       // Delay between turning off LEDs (100-150ms)
uint32_t last_off_time = 0;          // Last time an LED was turned off
uint8_t leds_to_turn_off = 0;


uint8_t toggle_delay_flag = 0;
uint8_t prev_button_state = 1; // Assume unpressed at start

// Timer delay tracking
//uint16_t current_delay_ms = 1000;



// --- Number of modes ---
const uint8_t num_modes = 3;

// --- Mode 1: left to right and back ---
const uint8_t mode1_pattern[] = {
    0b00000001, 0b00000010, 0b00000100, 0b00001000,
    0b00010000, 0b00100000, 0b01000000, 0b10000000,
    0b01000000, 0b00100000, 0b00010000, 0b00001000,
	0b00000100, 0b00000010
};
const uint8_t mode1_length = sizeof(mode1_pattern);

// --- Mode 2: inverse of mode 1 ---
const uint8_t mode2_pattern[] = {
    0b11111110, 0b11111101, 0b11111011, 0b11110111,
    0b11101111, 0b11011111, 0b10111111, 0b01111111,
    0b10111111, 0b11011111, 0b11101111, 0b11110111,
	0b11111011, 0b11111101,
};
const uint8_t mode2_length = sizeof(mode2_pattern);

// --- Mode 3: Sparkle ---
/*uint8_t current_led_state = 0b00000000;
uint8_t random_led_target = 0b00000000;
uint8_t random_state_step = 0;
uint32_t random_hold_counter = 0;*/
//uint8_t sparkle_pattern = 0;
uint8_t sparkle_phase = 0;           // 0 = hold, 1 = turning off
uint8_t sparkle_off_index = 0;       // which LED to turn off
uint16_t sparkle_hold_delay = 1000;  // random hold delay
uint8_t counter = 0;

// --- Shared state variables ---
uint8_t current_mode = 0;         // 1 = mode1, 2 = mode2, 3 = sparkle
uint8_t current_pattern_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void TIM16_IRQHandler(void);
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // TODO: Start timer TIM16

  HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // TODO: Check pushbuttons to change timer delay

	  uint8_t current_state = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
	  if (prev_button_state == 1 && current_state == 0) // falling edge: pressed
	  {
		  HAL_Delay(50);  // debounce delay

		  current_state = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
		  if (current_state == 0)
		  {
			  // Toggle the delay flag
			  toggle_delay_flag ^= 1;

			  // Update ARR value depending on delay
			  if (toggle_delay_flag == 0)
			  {
				  __HAL_TIM_SET_AUTORELOAD(&htim16, 1000 - 1);  // 1 second
			  }
			  else
			  {
				  __HAL_TIM_SET_AUTORELOAD(&htim16, 500 - 1);   // 0.5 second
			  }

			  // Re-initialize the counter to apply the new period immediately
			  __HAL_TIM_SET_COUNTER(&htim16, 0);

			  // Wait for button to be released (avoid multiple toggles)
			  while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) == 0);
			  HAL_Delay(50);  // debounce release
		  }

	  }
	  prev_button_state = current_state;


	  // Check for Mode buttons (active-low logic)
	  if (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_1) == 0) {
		  current_mode = 1;
	  } else if (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_2) == 0) {
		  current_mode = 2;
	  } else if (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_3) == 0) {
		  current_mode = 3;
	  }

	  // Execute current LED pattern
	  if (current_mode == 1) {
		  // Mode 1: Single LED moving back and forth
		  HAL_GPIO_WritePin(GPIOB, 0xFF, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOB, (1 << led_position), GPIO_PIN_SET);

		  led_position += direction;
		  if (led_position >= 7) direction = -1;
		  else if (led_position <= 0) direction = 1;
	  }

	  else if (current_mode == 2) {
		  // Mode 2: All LEDs ON except one
		  HAL_GPIO_WritePin(GPIOB, 0xFF, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, (1 << led_position), GPIO_PIN_RESET);

		  led_position += direction;
		  if (led_position >= 7) direction = -1;
		  else if (led_position <= 0) direction = 1;
	  }

	  //Attempt5
	  else if (current_mode == 3) {
	      uint32_t current_time = HAL_GetTick();

	      if (sparkle_pattern == 0) {
	          // Initialize new pattern
	          sparkle_pattern = rand() % 256;
	          HAL_GPIO_WritePin(GPIOB, sparkle_pattern, GPIO_PIN_SET);
	          leds_to_turn_off = sparkle_pattern;
	          next_arr_value = (rand() % 51) + 100;  // First delay: 100–150ms
	          __HAL_TIM_SET_AUTORELOAD(&htim16, next_arr_value - 1);
	          __HAL_TIM_SET_COUNTER(&htim16, 0);
	      }
	      else if (current_time - last_mode3_tick >= next_arr_value) {
	          // Turn off next LED
	          for (uint8_t i = 0; i < 8; i++) {
	              if ((leds_to_turn_off >> i) & 0x01) {
	                  HAL_GPIO_WritePin(GPIOB, (1 << i), GPIO_PIN_RESET);
	                  leds_to_turn_off &= ~(1 << i);
	                  next_arr_value = (rand() % 51) + 100;  // New delay
	                  __HAL_TIM_SET_AUTORELOAD(&htim16, next_arr_value - 1);
	                  __HAL_TIM_SET_COUNTER(&htim16, 0);
	                  last_mode3_tick = current_time;
	                  break;
	              }
	          }

	          // Reset if all LEDs are off
	          if (leds_to_turn_off == 0) sparkle_pattern = 0;
	      }
	  }




  /* USER CODE END 3 */
}

  void SystemDelay(void)
  {
	  HAL_Delay(50 + rand() % 101);
  }

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  NVIC_EnableIRQ(TIM16_IRQn);
  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  GPIO_InitStruct.Pin = Button0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = Button3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(Button3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED0_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TIM16_IRQHandler(void)
{
	// Acknowledge interrupt
	HAL_TIM_IRQHandler(&htim16);

	// TODO: Change LED pattern

	// second attmpt

	// Added the following

	 HAL_TIM_IRQHandler(&htim16);

}


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
