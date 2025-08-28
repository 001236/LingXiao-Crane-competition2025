#ifndef __SERVO_SM40BL_H
#define __SERVO_SM40BL_H

#include "stm32f4xx_hal.h"
#include "usart.h"

void servo1_maxspeed_to_direction(uint16_t direction);//默认ID:1,speed:90
void servo1_maxspeed_to_direction_cm(double direction_cm);//默认ID:1,speed:90

uint8_t servo1_is_run(void);//转动回复1，停下回复0

#endif
