#include "servo_SM40BL.h"
uint8_t uart2_rx_data[15];

//direction:0-32767
void servo1_maxspeed_to_direction(uint16_t direction)//默认ID:1,speed:90
{
	uint8_t	send_direction[13]={0xFF,0xFF,0x01,0x09,0x03,0x2A,0x00,0x00,0x00,0x00,0x5A,0x00,0xDD};
	
	send_direction[7] =  direction >> 8;//dir_high
	send_direction[6] = (direction << 8) >> 8;//dir_low
	
	send_direction[12]= ~(0x91 + send_direction[7] + send_direction[6]);//Check Sum
	HAL_UART_Transmit(&huart2, send_direction, 13, 0xFF);
	
	//HAL_UART_Receive(&huart1, uart1_rx_data, 6, 0xFF);
}

void servo1_maxspeed_to_direction_cm(double direction_cm)//默认ID:1,speed:90
{
	uint16_t direction = direction_cm* 372.41 + 3400;
	if(direction <= 900)
		direction = 900;
//	else if(direction >= 11700)
//		direction = 11700;
	servo1_maxspeed_to_direction(direction);
}

uint8_t servo1_is_run(void)//转动回复1，停下回复0
{
	uint8_t	send_direction[8]={0xFF,0xFF,0x01,0x04,0x02,0x42,0x01,0xB5};

	HAL_UART_Transmit(&huart2, send_direction, 8, 0xFF);
//判断接收值
	HAL_UART_Receive(&huart2, uart2_rx_data, 7, 0xFF);
	uint8_t state = 1;
	if(uart2_rx_data[6] == 0xFB)//停
		state = 0;
	
	return state;
}