#include "pid.h"

void PID_Init(PID_Value* PID, float Kp, float Ki, float Kd)// 定义PID初始化函数
{
	PID->Kp = Kp;	// 对这几个系数进行赋值
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->SP = 0.0f;
	PID->time_k_1 = 0;
	PID->err_k_1 = 0.0f;
	PID->err_I_k_1 = 0.0f;
}

void PID_SPinit(PID_Value* PID, float SP)//定义系统设定值初始化函数
{
	PID->SP = SP; // 赋值设定值
}

float PID_compute(PID_Value* PID, float FdBk)//进行PID运算
{
	float err = PID->SP - FdBk;// 计算误差值
	float time_k = HAL_GetTick();// 获取当前时间
	float delta_T = (time_k - PID->time_k_1) * 1.0e-3f;// 两次PID计算的时间间隔
	
	float err_I = (err - PID->err_k_1) / delta_T;// 计算微分值
	float err_D =PID->err_I_k_1 + (err + PID->err_k_1) * delta_T * 0.5f;// 计算积分值
	
	float P_output = PID->Kp * err; // 计算比例输出
	float I_output = PID->Ki * err_I; // 计算比例输出
	float D_output = PID->Kd * err_D; // 计算比例输出
	
	float CO = P_output + I_output + D_output;//计算最终PID输出
	
	PID->err_k_1 = err;// 对旧误差赋值
	PID->time_k_1 = time_k;// 对旧时间赋值
	PID->err_I_k_1 = err_I;// 对旧积分赋值
	
	return CO;
}
	