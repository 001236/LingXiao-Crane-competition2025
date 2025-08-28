/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Caculate.h"
#include "DJI.h"
//#include "uart_printf.c"
#include "wtr_can.h"
#include "wtr_uart.h"
#include "M2006.h"
#include "stdlib.h"
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
uint8_t rx_buffer[10];//接收数组
uint8_t rx_done = 0;//接收完成标志�?
uint8_t tx_once_flag = 0;//发�?�检验标志位
double ref;
int rx_index=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//float_to_byte place;//目标位置

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
  MX_CAN1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

	CANFilterInit(&hcan1);//用于初始化can1总线、开启过滤器，无法使用can2
	M2006_init();
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6,rx_buffer,10);
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//发�?�格式要求：'A' + float编码�?4位）
  while (1)
  {
		if(rx_done == 1 && rx_buffer[0] == 'A')//发�?�新位置要求，重置当前位置计�?
		{
//			place.byte[0] = rx_buffer[1];
//			place.byte[1] = rx_buffer[2];
//			place.byte[2] = rx_buffer[3];
//			place.byte[3] = rx_buffer[4];
			//set_M2006_position_cm(ref);
			rx_done = 0;
			tx_once_flag = 1;//�?次接收后只进行一次回�?
		}
		set_M2006_position_cm(ref);
		
		CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,hDJI[1].speedPID.output,hDJI[2].speedPID.output,hDJI[3].speedPID.output); // 自行选择�?要的电机ID填入
		
		if(M2006_done() == 1 && tx_once_flag == 1)//到位
		{
//			HAL_UART_Transmit_DMA(&huart1, "M2006 DONE", 11);//发�?�M2006 DONE\0
			HAL_UART_Transmit(&huart6,"1\r\n",3,0xff);
			tx_once_flag = 0;//复位tx_once_flag
		}
		HAL_Delay(5);// 循环时间，确定PID频率与发送频�?	反馈信息有直接�?�度数据，因此不用定时器也可以在该延时时间可变的前提下完成PID计算
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == USART1)
//	{
//		rx_done = 1;
//		HAL_UART_Receive_IT(&huart1, rx_buffer, 5);
//	}
//}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,uint16_t Size){
	if(huart==&huart6){//第一位设为标志位
		rx_done=1;
		receive_flag = rx_buffer[0];
		ref=atof(&rx_buffer[1]);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6,rx_buffer,10);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_HT);
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
