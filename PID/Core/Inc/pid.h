#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct	//定义PID结构体
{
	float Kp;	//比例系数
	float Ki;	//积分系数
	float Kd;	//微分系数
	float SP;	//设定值
	
	uint64_t time_k_1;//计算上次PID的运行时间
	float err_k_1;//存放上次误差的值
	float err_I_k_1;//上次积分值

} PID_Value;

void PID_Init(PID_Value* PID, float Kp, float Ki, float Kd);// 定义PID初始化函数
void PID_SPinit(PID_Value* PID, float SP);//定义系统设定值初始化函数
float PID_compute(PID_Value* PID, float FdBk);//进行PID运算

#endif
