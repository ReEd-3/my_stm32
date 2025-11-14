/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define BUFFER_LINE 256

extern uint8_t tx_buffer[BUFFER_LINE]; //发送缓冲区
extern uint8_t rx_buffer[BUFFER_LINE]; //接收缓冲区
extern uint32_t head; //头部位置
extern uint32_t datalen; //传输的数据长度
extern uint32_t all_datalen; //总的数据长度
extern uint8_t complete_flag;//结束标志

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
//	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET){// 如果发生空闲中断
//		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
//		__HAL_UART_CLEAR_IDLEFLAG(&huart1); //清除空闲中断的标记
//		HAL_DMA_Abort(huart1.hdmarx);

//		all_datalen = sizeof(rx_buffer) - __HAL_DMA_GET_COUNTER(huart1.hdmarx); //计算当前总数据长度
//		head = head + datalen; //计算头部位置
////		if(head >= BUFFER_LINE){ // 处理头部溢出
////			head = head - BUFFER_LINE;
////		}
////		if(all_datalen > head){ // 处理尾部溢出
//			datalen = all_datalen - head; //计算数据长度
////		}
////		else{
////			datalen = all_datalen + BUFFER_LINE - head;
////		}

//		for(int i = head; i < head + datalen; i++){ // 将rx数据写入tx
//			if(i < BUFFER_LINE){
//				tx_buffer[i - head] = rx_buffer[i];
//			}
//			else if(i > BUFFER_LINE){
//				tx_buffer[i - head] = rx_buffer[i - BUFFER_LINE];
//			}
//		}

//		complete_flag = 1; //设置结束标志
		
//		HAL_UART_Receive_DMA(&huart1, rx_buffer, sizeof(rx_buffer));//启动DMA接收数据
		
//	}

	if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET){ //如果发生空闲中断
		

	  __HAL_UART_CLEAR_IDLEFLAG(&huart1);  //清除空闲标志
		__HAL_DMA_DISABLE(huart1.hdmarx);  //暂停DMA的接收
		
		all_datalen = sizeof(rx_buffer) - __HAL_DMA_GET_COUNTER(huart1.hdmarx); //计算当前总数据长度

		head = head + datalen;
		if(head >= BUFFER_LINE){ // 处理头部溢出
			head = head - BUFFER_LINE;
		}
		if(all_datalen > head){ // 处理尾部溢出
			datalen = all_datalen - head; //计算数据长度
		}
		else{
			datalen = all_datalen + BUFFER_LINE - head;
		}
		
		for(int i = head; i < head + datalen; i++){ // 将rx数据写入tx
			if(i < BUFFER_LINE){
				tx_buffer[i - head] = rx_buffer[i];
			}
			else if(i > BUFFER_LINE){
				tx_buffer[i - head] = rx_buffer[i - BUFFER_LINE];
			}
		}
		complete_flag = 1;//完成标记，允许进入主循环

		
	}
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
