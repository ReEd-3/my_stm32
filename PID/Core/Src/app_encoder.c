#include "app_encoder.h"
#include "tim.h"

#define ENCODER_PPR  11    // 编码器线数（一圈的脉冲数，不含倍频）

static volatile int64_t encoder = 0; // 表示电机编码器的值

void Encoder_Init()
{
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 开启编码器
}

float GetEncoder(uint32_t sample_time){
	float encoder;
	float Omega;
	encoder = (int16_t)__HAL_TIM_GET_COUNTER(&htim4); // 获取编码器记数
	__HAL_TIM_SET_COUNTER(&htim4, 0); // 编码器计数清零
	Omega = (encoder / ENCODER_PPR) * (1000 / sample_time);
	return Omega;
}

