#include "motor_ctrl.h"
#include "pid.h"
#include "app_encoder.h"
#include "tim.h"

#define U_0 1.0   // 总电压

static PID_Value the_pid;// 声明PID变量

void Motor_Init(float SP)
{
	PID_Init(&the_pid, 1.0, 1.0, 1.0);
	PID_SPinit(&the_pid, SP);
}	

void Motor_Excu()
{
	//20ms执行一次读取
	static uint32_t nxt = 0;
	while(HAL_GetTick() - nxt <= 20);
	nxt = nxt + 20;
	//获取角速度
	float omega = GetEncoder(20) ;
	//进行输出值计算
	float Ua = PID_compute(&the_pid, omega);
	//计算占空比
	float duty = (Ua / U_0) * 100.0;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);// 设置转速
}

