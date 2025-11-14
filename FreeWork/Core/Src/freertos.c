/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define BUFFER_LINE 256
#include "math.h"

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern uint32_t head; //头部位置
extern uint8_t tx_buffer[BUFFER_LINE]; //发送缓冲区
extern uint8_t rx_buffer[BUFFER_LINE]; //接收缓冲区
extern uint32_t all_datalen; // 数据总长
extern uint32_t datalen; //    数据长度
extern uint8_t complete_flag;//结束标志

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for B_LED */
osThreadId_t B_LEDHandle;
const osThreadAttr_t B_LED_attributes = {
  .name = "B_LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DMA_IO */
osThreadId_t DMA_IOHandle;
const osThreadAttr_t DMA_IO_attributes = {
  .name = "DMA_IO",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartB_LED(void *argument);
void StartDMA_IO(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of B_LED */
  B_LEDHandle = osThreadNew(StartB_LED, NULL, &B_LED_attributes);

  /* creation of DMA_IO */
  DMA_IOHandle = osThreadNew(StartDMA_IO, NULL, &DMA_IO_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartB_LED */
/**
  * @brief  Function implementing the B_LED thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartB_LED */
void StartB_LED(void *argument)
{
  /* USER CODE BEGIN StartB_LED */
  /* Infinite loop */
  for(;;)
  {
    int t;
		for(int i=0;i<100;i++){
			t=50*sin(2 * i*3.1415/100)+50;
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,t);
			HAL_Delay(10);
		}
		
		HAL_Delay(100);//等待100ms
  }
  /* USER CODE END StartB_LED */
}

/* USER CODE BEGIN Header_StartDMA_IO */
/**
* @brief Function implementing the DMA_IO thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDMA_IO */
void StartDMA_IO(void *argument)
{
  /* USER CODE BEGIN StartDMA_IO */



  /* Infinite loop */
  for(;;)
  {
		
//		if(complete_flag){ // 进入了循环
//			complete_flag = 0; // 结束标志归零
//			HAL_UART_Transmit(&huart1, tx_buffer, datalen, 2000); // 发送信息
//			HAL_UART_Transmit_DMA(&huart1, &rx_buffer[head], datalen); //发送收到的数据
////			__HAL_DMA_ENABLE(huart1.hdmarx); //重新启动DMA接收
//			HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));//启动DMA接收数据
//		}
//		osDelay(20);
		
		
		
		
		if(complete_flag){//如果空闲中断已经处理完数据
			


			complete_flag = 0; //清除完成标志
			HAL_UART_Transmit_DMA(&huart1, tx_buffer, datalen); //发送收到的数据
//			HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));//启动DMA接收数据
			__HAL_DMA_ENABLE(huart1.hdmarx); //重新启动DMA接收

		}
		HAL_Delay(20);
		

		
  }
  /* USER CODE END StartDMA_IO */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

