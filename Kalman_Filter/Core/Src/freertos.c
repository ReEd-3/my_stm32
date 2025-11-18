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

#define ACC_R 0.00000013 // 加速度的测量方差
#define GYR_R 1.388542 // 角速度的测量方差
#define ACC_Q 0.0000000035 // 加速度的过程方差
#define GYR_Q 10.0 // 角速度的过程方差

#include "stdio.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

double ACC_P_; // 加速度先验协方差
double ACC_P = 0; // 加速度后验协方差
double ACC_K; // 加速度卡尔曼增益
double GYR_P_; // 角速度先验协方差
double GYR_P = 0; // 角速度后验协方差
double GYR_K; // 角速度卡尔曼增益

typedef struct{
	double X_; // 加速度预测值
	double Y_;
	double Z_;
	double X; // 加速度最优值
	double Y;
	double Z;
	double XZ; // 加速度观测值
	double YZ;
	double ZZ;

}ACCTYPE;
ACCTYPE Acc = {0, 0, 0, 0, 0, 0, 0, 0, 0};

typedef struct{
	double X_; // 角速度预测值
	double Y_;
	double Z_;
	double X; // 角速度最优值
	double Y;
	double Z;
	double XZ; // 角速度观测值
	double YZ;
	double ZZ;
}GYRTYPE;
GYRTYPE Gyr = {0, 0, 0, 0, 0, 0, 0, 0, 0};

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

void Kalman_Filter(double raw_x, double raw_y, double raw_z){
	// 获取预测值
	Acc.X_ = Acc.X;
	Acc.Y_ = Acc.Y;
	Acc.Z_ = Acc.Z;
	// 计算先验协方差
	ACC_P_ = ACC_P + ACC_Q;
	// 计算卡尔曼增益
	ACC_K = ACC_P_/(ACC_P_ + ACC_R);
	// 计算最优值
	Acc.X = Acc.X_ + ACC_K * (raw_x - Acc.X_);
	Acc.Y = Acc.Y_ + ACC_K * (raw_y - Acc.Y_);
	Acc.Z = Acc.Z_ + ACC_K * (raw_z - Acc.Z_);
	// 计算后验协方差
	ACC_P = (1 - ACC_K) * ACC_P_;
}
char buffer[128];
int length;
uint8_t MPUData[14];
uint8_t StarAddress = 0x3b;

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
		HAL_I2C_Mem_Read(&hi2c1, 0XD1, StarAddress, I2C_MEMADD_SIZE_8BIT, MPUData, 14, 50); // 读取MPU6050数据
		// 读取三维加速度
		short int ACC_X1 = ((MPUData[0]<<8) | MPUData[1]);
		Acc.XZ = (double)ACC_X1/16384;
		short int ACC_Y1 =((MPUData[2]<<8)| MPUData[3]);
		Acc.YZ = (double)ACC_Y1/16384;
		short int ACC_Z1=((MPUData[4]<<8) | MPUData[5]);
		Acc.ZZ = (double)ACC_Z1/16384;
		// 读取三维角速度
		short int GYR_X1= ((MPUData[8]<<8) | MPUData[9]);
		Gyr.XZ = (double)GYR_X1/65.5;
		short int GYR_Y1 = ((MPUData[10]<<8) | MPUData[11]);
		Gyr.YZ = (double)GYR_Y1/65.5;
		short int GYR_Z1 = ((MPUData[12]<<8) | MPUData[13]);
		Gyr.ZZ = (double)GYR_Z1/65.5;

		Kalman_Filter(Acc.XZ,Acc.YZ,Acc.ZZ); // 进行卡尔曼滤波



		length = sprintf(buffer, "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n", Acc.X, Acc.Y, Acc.Z,Acc.XZ,Acc.YZ,Acc.ZZ);
		HAL_UART_Transmit(&huart1, (uint8_t*)buffer, length, 100);

    osDelay(5);
  }
  /* USER CODE END StartMPU */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

