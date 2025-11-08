/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define DEBOUNCE 20
//#define LONG 1000

typedef enum{
	STT_NULL = 0,
	STT_PRS,
	STT_LONG,
	STT_WAIT
} KeyState;

KeyState keystt = STT_NULL;

typedef enum{
	EVT_NULL = 0,
	EVT_CLK,
	EVT_LONG,
	EVT_DOUBLE
} KeyEvent;

KeyEvent keyevt = EVT_NULL;

typedef enum{
	SET_A,
	SET_B,
	SET_C
} ServoSet;

ServoSet srvset = SET_A;

//void Key_Click_Scan(void){
//  
//	static uint32_t nowtime = 0;
//	nowtime = HAL_GetTick();//获取当前时间
//	
//  uint16_t keyread = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);//识别输入口的状态
//  
//	if(keystt == STT_NULL){
//		if(keyread == GPIO_PIN_RESET){
//			keystt = STT_PRS;
//		}
//	}
//	
//	
//	
//	else if(keystt == STT_PRS){  
//		if(nowtime - presstime >= DEBOUNCE){  // 按键按下
//			             // 消抖延时
//			keyread = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
//			if(keyread == GPIO_PIN_RESET){
//				keystt = STT_LONG;
//				presstime = nowtime;//记录按下的时间
//				
//			}
//			else{
//				keystt = STT_NULL;
//			}
//		}
//	}
//	
//	else if(keystt == STT_LONG){
//		if(keyread == GPIO_PIN_SET){
//			if(nowtime - presstime < LONG){
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 翻转LED
//			}
//		}
//  }
//}
#define LONG 1000
#define DOUBLE 500

uint32_t nowtime = 0; 
uint32_t presstime = 0; 
uint32_t releasetime = 0;

void Key_Click_Scan(void){
	nowtime = HAL_GetTick();
  uint16_t keyread = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
  
  if(keystt == STT_NULL){  // 按键按下
    if(keyread == GPIO_PIN_RESET){           // 第一次检测到按下
      HAL_Delay(20);              // 消抖延时
      keyread = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
      if(keyread == GPIO_PIN_RESET){ // 确认按下
				presstime = nowtime;
        keystt = STT_PRS;          // 标记按键已处理
      }
    }
  }
  else if(keystt == STT_PRS){//按下后检测是否长按
		if(keyread == GPIO_PIN_SET){//释放时
			if(nowtime - presstime < LONG){//如果小于长按时长
			  keystt = STT_WAIT;
				releasetime = nowtime;

			}
			else{
			 keyevt = EVT_LONG;
       keystt = STT_NULL;		
			}
		}
      // 按键释放，重置状态
  }
	else if(keystt == STT_WAIT){
		
		if(nowtime - releasetime < DOUBLE){
			if(keyread == GPIO_PIN_RESET){
				HAL_Delay(20);              // 消抖延时
				keyread = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
				if(keyread == GPIO_PIN_RESET){ // 确认按下
					keystt = STT_NULL;
					keyevt = EVT_DOUBLE;
					HAL_Delay(50);
				}
			}
		}
		else{
			keystt = STT_NULL;
			keyevt = EVT_CLK;
		}
	}
}

void Key_Event(){
	if(keyevt!= EVT_NULL){
		if(keyevt == EVT_CLK){
			srvset = SET_A;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			keyevt = EVT_NULL;
		}
		else if (keyevt == EVT_LONG){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			srvset = SET_B;
			keyevt = EVT_NULL;
		}
		else if (keyevt ==EVT_DOUBLE){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			srvset = SET_C;
			keyevt = EVT_NULL;
		}
	}
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
  MX_GPIO_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint16_t readkey = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		Key_Click_Scan();
		Key_Event();
		if(srvset == SET_A){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50);
		}
		else if(srvset == SET_B){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 150);
		}
		else if(srvset == SET_C){
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250);
		}
		HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
