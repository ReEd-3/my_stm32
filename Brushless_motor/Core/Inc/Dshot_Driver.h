#ifndef DSHOT_DRIVER_H
#define DSHOT_DRIVER_H

#include "stdint.h"

uint16_t Data_packet(uint16_t speed, uint8_t request_flag);
void Dshot_Tx(uint16_t packet);
void Dshot_Set_Speed(uint16_t speed);

#endif
