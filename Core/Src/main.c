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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdbool.h"
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
//Testing variables
uint32_t g_distance=0;
uint32_t distance_recv;
volatile uint32_t g_pulselen=0;
volatile bool     g_new_distance_data=false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* Servo motor interfaces */
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (float) ((x*1.0 - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
void PWM_Init(void);
void PWM_SetDuty(uint8_t duty);
void Servo_SetAngle(int8_t angle);
void Servo_Reset(void);

void SupplyGate_Open(void)
{
	/* From 0 deg - > 90 Deg */
	for(int8_t deg = 0; deg < 90; deg +=5)
	{
		Servo_SetAngle(deg);
		HAL_Delay(100);
	}
}


void SupplyGate_Close(void)
{
	/* From 90 deg - > 0 Deg */
	for(int8_t deg = 90; deg > 0; deg -= 5)
	{
		Servo_SetAngle(deg);
		HAL_Delay(100);
	}
}




/* HC_SR04 ultra sonic sensor interface */
void HC_SR04_TriggerPulse(void);
bool HC_SR04_ReadData(uint32_t* p_data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
volatile uint16_t g_duty=0;
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
	MX_TIM2_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim4);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		SupplyGate_Open();
		HAL_Delay(5000);
		SupplyGate_Close();

		PWM_SetDuty(g_duty);
		HAL_Delay(1000);
//		Servo_SetAngle(g_duty);
//		HC_SR04_ReadData(&distance_recv);
//		HAL_Delay(1000);
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void PWM_Init(void)
{
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim4);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
}

void PWM_SetDuty(uint8_t duty)
{
  if(duty == 0)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  }
  else if (duty >= 100)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 200);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, (duty*200 / 100));
  }
}

void Servo_SetAngle(int8_t angle)
{
  uint8_t duty_set = map(angle, -90, 90, 5, 10); // -90 deg = 1ms = 5% duty cycle, 90 = 2ms = 10% duty cycle
  PWM_SetDuty(duty_set);
}

void HC_SR04_TriggerPulse(void)
{
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);
}

bool HC_SR04_ReadData(uint32_t* p_data)
{
   HC_SR04_TriggerPulse();
   //Wait for respond
   HAL_Delay(500);
   if( false == g_new_distance_data)
   {
      return false;
   }
  else
   {
	  if(g_pulselen < 30000)
	  {
		  *p_data = g_pulselen * 10 / 575;
	  }
	  g_new_distance_data = false;

   }

   return true;
}

void EXTI0_IRQHandler(void)
{
  static bool rising_edge_detected=false;
  if(ECHO_Pin == HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) ) //Rising
  {
    //Reset timer
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    HAL_TIM_Base_Start(&htim2);

    rising_edge_detected = true;

  }
  else //Falling edge
  {
    if(true == rising_edge_detected)
    {
      // Get timer value
      g_pulselen = __HAL_TIM_GET_COUNTER(&htim2);
      g_new_distance_data = true;

    }

    rising_edge_detected = false;
    HAL_TIM_Base_Stop(&htim2);
  }

  HAL_GPIO_EXTI_IRQHandler(ECHO_Pin);
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

