/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "PID.h"
#include "motor.h"
#include "stdlib.h"
#include "string.h"
#include "ctype.h"
#include "stp_23l.h"
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

/* USER CODE BEGIN PV */
uint8_t receive_flag = '0';
uint8_t receiveData[10];
double ref;
double ref1;
motor_data motor1;
motor_data motor2;
double pfdb;
char arr[6];
int rxindex=0;
u16 receive_cnt;//¼Ƌ㳉¹¦½ӊՊý¾ݖ¡´Ί�?
u8 confidence;
u16 distance,noise,reftof;
u32 peak,intg;
u8 dis;
extern LidarPointTypedef Pack_Data[12];/* À״ｓʕµĊý¾ݴ¢´攚բ¸ö±䁿֮�? */
extern LidarPointTypedef Pack_sum; 
uint8_t stp_Receive_buf[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
motor_init(htim2,htim1,TIM_CHANNEL_1,&motor1);
motor_init(htim3,htim1,TIM_CHANNEL_2,&motor2);
HAL_TIM_Base_Start_IT(&htim1);//�?启中断时不能同时�?启时�?,否则不进中断
motor_start(&motor1);
motor_start(&motor2);
HAL_UARTEx_ReceiveToIdle_DMA(&huart3,receiveData,10);
HAL_UART_Receive_IT(&huart2,stp_Receive_buf,sizeof(stp_Receive_buf));
__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
//set_pidp_ref(&motor1,285/8.3/3.14159265358);
//		  set_pidp_ref(&motor2,285/8.3/3.14159265358);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)//如果加传感器，在传感器电平变化的时�?�只�?要修改位置式pid的反馈�?�，即实际已经行进多�?
  {
		set_motor_direction(&motor2,AIN1_GPIO_Port,AIN1_Pin,AIN2_GPIO_Port,AIN2_Pin);
		set_motor_direction(&motor1,BIN1_GPIO_Port,BIN1_Pin,BIN2_GPIO_Port,BIN2_Pin);
		if(receive_flag =='a'){
			ref1=ref/8.3/3.14159265358;//ref参数单位为cm
		  set_pidp_ref(&motor1,ref1);
		  set_pidp_ref(&motor2,ref1);
			receiveData[0]='0';
			receive_flag='0';
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim1){
		motor_pid_control(&motor2);
		motor_pid_control(&motor1);
		pfdb=(motor1.motor_pid_p.fdb+motor2.motor_pid_p.fdb)/2;
		motor1.motor_pid_p.fdb=motor2.motor_pid_p.fdb=pfdb;
	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,uint16_t Size){
	if(huart==&huart3){//第一位设为标志位
		receive_flag = receiveData[0];
		ref=atof(&receiveData[1]);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,receiveData,10);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	static uint8_t state = 0;			
	static uint8_t crc = 0;				
	static uint8_t cnt = 0;			
	static uint8_t PACK_FLAG = 0; 
	static uint8_t data_len  = 0; 
	static uint32_t timestamp = 0;
	static uint8_t state_flag = 1;
	uint8_t temp_data;
	if(huart -> Instance == USART2)
	{
			temp_data=stp_Receive_buf[0];	
			if(state< 4) 																					
				{                                          
						if(temp_data == HEADER) state ++;
						else state = 0;
				}
				else if(state<10&&state>3)
				{
						switch(state)
						{
								case 4:   
									if(temp_data == device_address)             
									{							
													state ++;
													crc = crc + temp_data;									
													break;        
									} 
									else state = 0,crc = 0;
								case 5:   
									if(temp_data == PACK_GET_DISTANCE)					
									{  
													PACK_FLAG = PACK_GET_DISTANCE;
													state ++;	
													crc = crc + temp_data;	
													break;									
									}		 

									else if(temp_data == PACK_RESET_SYSTEM) 		 
									{
													PACK_FLAG = PACK_RESET_SYSTEM;
													state ++; 
													crc = crc + temp_data;	
													break;	
									}
									else if(temp_data == PACK_STOP)							 
									{ 
													PACK_FLAG = PACK_STOP;
													state ++; 
													crc = crc + temp_data;	
													break;
									}
									else if(temp_data == PACK_ACK)							 
									{  
													PACK_FLAG = PACK_ACK;
													state ++;
													crc = crc + temp_data;	
													break;
									}			 				 
									else if(temp_data == PACK_VERSION)					 
									{ 
													PACK_FLAG = PACK_VERSION,
													state ++,
													crc = crc + temp_data;	   	     
													break;
									}
									else state = 0,crc = 0;
								case 6: if(temp_data == chunk_offset)          
												{  
													state ++;
													crc = crc + temp_data;
													break; 	  
												}	
												else state = 0,crc = 0;
								case 7: if(temp_data == chunk_offset)
												{  
													state ++;
													crc = crc + temp_data;
													break;
												}
												else state = 0,crc = 0;
								case 8: 
										data_len = (u16)temp_data;								 
										state ++; 
										crc = crc + temp_data;
										break;																			 
								case 9: 
										data_len = data_len + ((u16)temp_data<<8); 			 
										state ++;
										crc = crc + temp_data;
										break; 
								default: break;
						}
				}
				else if(state == 10 ) state_flag = 0;                     
				if(PACK_FLAG == PACK_GET_DISTANCE&&state_flag == 0)     
				{
						if(state>9)
						{
								if(state<190)
								{
										static uint8_t state_num;
										state_num = (state-10)%15;
										switch(state_num)
										{
												case 0: 
													Pack_Data[cnt].distance = (uint16_t)temp_data ;				 
													crc = crc + temp_data;
													state++;
													break;        
												case 1: 
													Pack_Data[cnt].distance = ((u16)temp_data<<8) + Pack_Data[cnt].distance;	
													crc = crc + temp_data;
													state++;
													break; 
												case 2:
													Pack_Data[cnt].noise = (u16)temp_data;				 
													crc = crc + temp_data;
													state++;
													break; 
												case 3:
													Pack_Data[cnt].noise = ((u16)temp_data<<8) + Pack_Data[cnt].noise;				
													crc = crc + temp_data;
													state++;
													break; 
												case 4:
													Pack_Data[cnt].peak = (u32)temp_data;				 										
													crc = crc + temp_data;
													state++;
													break; 
												case 5:
													Pack_Data[cnt].peak = ((u32)temp_data<<8) + Pack_Data[cnt].peak;
													crc = crc + temp_data;
													state++;
													break; 
												case 6:
													Pack_Data[cnt].peak = ((u32)temp_data<<16) + Pack_Data[cnt].peak;	
													crc = crc + temp_data;
													state++;
													break; 
												case 7:
													Pack_Data[cnt].peak = ((u32)temp_data<<24) + Pack_Data[cnt].peak;				   
													crc = crc + temp_data;
													state++;
													break; 
												case 8:
													Pack_Data[cnt].confidence = temp_data;				 
													crc = crc + temp_data;
													state++;
													break; 
												case 9:
													Pack_Data[cnt].intg = (u32)temp_data;															
													crc = crc + temp_data;
													state++;
													break; 
												case 10:
													Pack_Data[cnt].intg = ((u32)temp_data<<8) + Pack_Data[cnt].intg;
													crc = crc + temp_data;
													state++;
													break; 
												case 11:
													Pack_Data[cnt].intg = ((u32)temp_data<<16) + Pack_Data[cnt].intg;
													crc = crc + temp_data;
													state++;
													break; 
												case 12:
													Pack_Data[cnt].intg = ((u32)temp_data<<24) + Pack_Data[cnt].intg;				  	 
													crc = crc + temp_data;
													state++;
													break; 
												case 13:
													Pack_Data[cnt].reftof = (int16_t)temp_data;				 							
													crc = crc + temp_data;
													state++;
													break; 
												case 14:
													Pack_Data[cnt].reftof = ((int16_t)temp_data<<8) +Pack_Data[cnt].reftof;			
													crc = crc + temp_data;
													state++;
													cnt++;							 
													break; 
												default: break;
										}
							}
										
										if(state == 191) timestamp = temp_data,state++,crc = crc + temp_data;
										else if(state == 192) timestamp = ((u32)temp_data<<8) + timestamp,state++,crc = crc + temp_data; 
										else if(state == 193) timestamp = ((u32)temp_data<<16) + timestamp,state++,crc = crc + temp_data;
										else if(state == 194) timestamp = ((u32)temp_data<<24) + timestamp,state++,crc = crc + temp_data; 
										else if(state==195)
										{
													if(temp_data == crc) 
													{
															data_process();  	
															receive_cnt++;	 	 
														  if(receive_cnt>1) receive_cnt=1;
													}
													//distance = Pack_Data[0].distance;
													crc = 0;
													state = 0;
													state_flag = 1;
													cnt = 0; 							
										}
										 if(state == 190) state++;
							
						}
				}
			HAL_UART_Receive_IT(&huart2,stp_Receive_buf,sizeof(stp_Receive_buf));
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
