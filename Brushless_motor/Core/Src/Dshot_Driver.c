#include "Dshot_Driver.h"
#include "stm32f1xx_hal.h"
#include "stdint.h"

#define BIT_1 6 // 位1占空比
#define BIT_0 3 // 位0占空比
#define SPEED_MIN 48 // 最小速度
#define SPEED_MAX 2047 // 最大速度
#define DSHOT_FRAME_LENGTH 18 // 数据帧长度
#define PWM_CHANNEL TIM_CHANNEL_1 // 通道

uint16_t dshot_dma_buffer[DSHOT_FRAME_LENGTH] = {0};
extern TIM_HandleTypeDef htim4;

// 获取数据帧
uint16_t Data_packet(uint16_t speed, uint8_t request_flag){
  uint16_t packet = 0; // 存放数据包
	packet = (speed << 1) | (request_flag ? 1 : 0); // 写入数据和是否请求位
	uint16_t packet_cp = packet;
  uint16_t crc = 0;
	for (int i = 0; i < 3; i++){
	  crc ^= packet_cp; // 进行分组异或，得出校验码
		packet_cp >>= 4;
	}
	crc &= 0x0F; // 取低四位
	return ((packet << 4) | crc);
}

void Dshot_Tx(uint16_t packet){
	for(int i = 0; i < 16; i++){
	  if(packet & 0x8000){
		  dshot_dma_buffer[i] = BIT_1;
		}else{
		  dshot_dma_buffer[i] = BIT_0;
		}
		packet <<= 1;
	}
	dshot_dma_buffer[16] = 0;
	dshot_dma_buffer[17] = 0;
  HAL_TIM_PWM_Start_DMA(&htim4, PWM_CHANNEL, (uint32_t *)dshot_dma_buffer, DSHOT_FRAME_LENGTH);
}

void Dshot_Set_Speed(uint16_t speed){
  uint16_t packet = Data_packet(speed, 0);
	Dshot_Tx(packet);
}
