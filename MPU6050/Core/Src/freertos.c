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

#define MPU6050_ADDR 0xD0 // MPU6050的设备地址
#define GYRO_CONFIG_REG 0x1B // 陀螺仪配置寄存器
#define ACCEL_CONFIG_REG 0x1C // 加速度计配置寄存器
#define ACCEL_XOUT_H_REG 0x3B //加速度计输出寄存器
#define GYRO_XOUT_H_REG 0x43 // 陀螺仪数据输出寄存器
#define PWR_MGMT_1_REG 0x6B // 电源管理寄存器
#define WHO_AM_I_REG 0x75 // 设备ID寄存器

#include "stdio.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;


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
/* Definitions for MPU */
osThreadId_t MPUHandle;
const osThreadAttr_t MPU_attributes = {
  .name = "MPU",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMPU(void *argument);

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
  /* creation of MPU */
  MPUHandle = osThreadNew(StartMPU, NULL, &MPU_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMPU */
void MPU6050_INIT(){

	HAL_Delay(1000);//   延时*********这里要增加***原本为100

	uint8_t SendAddress = 0x6b;
	uint8_t SendData = 0x00; //解除休眠状态
	HAL_I2C_Mem_Write(&hi2c1,0xD1,SendAddress,1,&SendData,1,0xff);
	SendAddress = 0x19;//采样率分频器
	SendData = 0x07;
	HAL_I2C_Mem_Write(&hi2c1,0xD1,SendAddress,1,&SendData,1,0xff);
	SendAddress = 0x1A;//低通滤波器
	SendData = 0x06;
	HAL_I2C_Mem_Write(&hi2c1,0xD1,SendAddress,1,&SendData,1,0xff);
	SendAddress = 0x1B;//陀螺仪
	SendData = 0x08; //± 500 °/s
	HAL_I2C_Mem_Write(&hi2c1,0xD1,SendAddress,1,&SendData,1,0xff);
	SendAddress = 0x1C;//加速度计
	SendData = 0x00; //± 2g
	HAL_I2C_Mem_Write(&hi2c1,0xD1,SendAddress,1,&SendData,1,0xff);
}

uint8_t MPUData[14];
uint8_t StarAddress = 0x3b;
double ACC_X = 0.0, ACC_Y = 0.0, ACC_Z = 0.0;
double GYR_X = 0.0, GYR_Y = 0.0, GYR_Z = 0.0;



/**
  * @brief  Function implementing the MPU thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMPU */
void StartMPU(void *argument)
{
  /* USER CODE BEGIN StartMPU */
	MPU6050_INIT();
  /* Infinite loop */
  for(;;)
  {
		HAL_I2C_Mem_Read(&hi2c1, 0XD1, StarAddress, I2C_MEMADD_SIZE_8BIT, MPUData, 14, 50);
		short int ACC_X1 = ((MPUData[0]<<8) | MPUData[1]);ACC_X = (double)ACC_X1/16384;
		short int ACC_Y1 =((MPUData[2]<<8)| MPUData[3]);ACC_Y = (double)ACC_Y1/16384;
		short int ACC_Z1=((MPUData[4]<<8) | MPUData[5]);ACC_Z = (double)ACC_Z1/16384;
		short int GYR_X1= ((MPUData[8]<<8) | MPUData[9]);GYR_X = (double)GYR_X1/65.5;
		short int GYR_Y1 = ((MPUData[10]<<8) | MPUData[11]);GYR_Y = (double)GYR_Y1/65.5;
		short int GYR_Z1 = ((MPUData[12]<<8) | MPUData[13]);GYR_Z = (double)GYR_Z1/65.5;
		static char buffer[128];
		static int length;

		// 打印加速度和陀螺仪数据

		length = sprintf(buffer, "%.3f,%.3f,%.3f\n,%.2f,%.2f,%.2f\n", ACC_X, ACC_Y, ACC_Z,GYR_X, GYR_Y, GYR_Z);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, length, 100);

//		// 打印陀螺仪数据
//		length = sprintf(buffer, , );
//		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, length, 100);


		
		
    osDelay(100);
  }
  /* USER CODE END StartMPU */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

