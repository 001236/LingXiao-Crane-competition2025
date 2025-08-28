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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Delay_us.h"
#include "stdlib.h"
#include "string.h"
#include "ctype.h"
#include "motor.h"
#include "math.h"
#define pi 3.14159265358979323846264
#define THRESHOLD_ANGLE_RAD (10.0/180*pi)
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
uint8_t receiveData[10];
double ref;
uint8_t receive_flag = '0';
 float current_pos;
 float last_angle=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MiAngleControlOn(int angle){
    float target_res = angle / 180.0 * pi;
    current_pos = mi_motor[0].Angle/ 180.0 * pi; // 假设这是获取当前电机角度的方法
	  
    // 计算当前位置与目标位置的绝对误差
    float error_abs = fabs(target_res - current_pos);
    if (error_abs > THRESHOLD_ANGLE_RAD) {
        // 远离目标时：启动慢，运动平缓
			while(error_abs > THRESHOLD_ANGLE_RAD){
				motor_controlmode(&mi_motor[0], 0, target_res, 0, 3.0, 1.8); // 较低的 kp 和 kd
				error_abs = fabs(target_res - mi_motor[0].Angle/ 180.0 * pi);
				HAL_Delay(1);
			}
        
    }
		
		if(error_abs <= THRESHOLD_ANGLE_RAD){
        // 接近目标时：加快收敛，提高末端速度和定位精度
       motor_controlmode(&mi_motor[0], 0, target_res, 0, 25.0, 25.0); // 较高的 kp 和 kd
			 last_angle=target_res;
    }
}

void MiAngleControlOff(int angle){
    float target_res = angle / 180.0 * pi;
    current_pos = mi_motor[0].Angle/ 180.0 * pi; // 假设这是获取当前电机角度的方法
	  
    // 计算当前位置与目标位置的绝对误差
    float error_abs = fabs(target_res - current_pos);
    if (error_abs > THRESHOLD_ANGLE_RAD) {
        // 远离目标时：启动慢，运动平缓
			while(error_abs > THRESHOLD_ANGLE_RAD){
				motor_controlmode(&mi_motor[0], 0, target_res, 0, 3.0, 1.8); // 较低的 kp 和 kd
				error_abs = fabs(target_res - mi_motor[0].Angle/ 180.0 * pi);
				HAL_Delay(1);
			}
        
    }
		
		if(error_abs <= THRESHOLD_ANGLE_RAD){
        // 接近目标时：加快收敛，提高末端速度和定位精度
       motor_controlmode(&mi_motor[0], 0, target_res, 0, 25.0, 25.0); // 较高的 kp 和 kd
			 last_angle=target_res;
    }
}
void bujin(int dir,double posi,int fuzai){//0下，1�?,单位：cm
	dir=(dir==0);
	HAL_GPIO_WritePin(GPIOB,DIRECT_Pin,dir);//货架箱子提手高度42-43cm；铝管架离地高度大概�?44cm，算轨道
	posi=posi/(0.628*30)*2800*4;
	if(fuzai==1){//1表示有负载，0表示无负�?
	 for(int i=0;i<posi;++i){
	   HAL_GPIO_WritePin(GPIOB,PUL_Pin,0);
	    Delay_us(30);
	    HAL_GPIO_WritePin(GPIOB,PUL_Pin,1);
	    Delay_us(30);
	  }
	}else if(fuzai==0&&dir==0){
		for(int i=0;i<posi;++i){
			HAL_GPIO_WritePin(GPIOB,PUL_Pin,0);
	    Delay_us(25);
	    HAL_GPIO_WritePin(GPIOB,PUL_Pin,1);
	    Delay_us(25);
		}
	}else if(fuzai==0&&dir==1){
		for(int i=0;i<posi;++i){
			HAL_GPIO_WritePin(GPIOB,PUL_Pin,0);
	    Delay_us(25);
	    HAL_GPIO_WritePin(GPIOB,PUL_Pin,1);
	    Delay_us(25);
		}
	}
}
void get(double posi_cm){
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,1825);//打开�?
	HAL_Delay(500);
	bujin(0,posi_cm,0);//下降
  HAL_Delay(100);
  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,1225);//收爪
  HAL_Delay(500);
  bujin(1,posi_cm,1);//上升
	HAL_Delay(10);
}
void place(double down_cm){
	bujin(0,down_cm,1);
  HAL_Delay(500);
  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,1825);
  HAL_Delay(800);
	bujin(1,5,0);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,1225);
	bujin(1,down_cm-5,0);
  HAL_Delay(10);
}

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
  MX_USART1_UART_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
		CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can_filter_st.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan, &can_filter_st);
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	HAL_TIM_Base_Start_IT(&htim2);//定时中断初始�?


	HAL_Delay(500);              //�?定时间延�? 等待电机初始化完�? 
	init_cybergear(&mi_motor[0], 0x7F, Motion_mode);//小米电机 启动!
  set_zeropos_cybergear(&mi_motor[0]);
HAL_GPIO_WritePin(GPIOB,ENABLE_Pin,0);
HAL_TIM_Base_Init(&htim1);
HAL_TIM_Base_Init(&htim3);
HAL_TIM_Base_Start(&htim3);
HAL_TIM_PWM_Init(&htim3);
HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
HAL_UARTEx_ReceiveToIdle_DMA(&huart1,receiveData,10);
__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
MiAngleControlOff(0);
HAL_Delay(5);
MiAngleControlOff(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	/**
		* @brief          小米运控模式指令
		* @param[in]      Motor:  目标电机结构使
		* @param[in]      torque: 力矩设置[-12,12] N*M
		* @param[in]      MechPosition: 位置设置[-12.5,12.5]rad				这里12.5昿4pi的近似忼，但我发现用近似忼控制转角好像更准确？？＿
		* @param[in]      speed: 速度设置[-30,30] rad/s
		* @param[in]      kp: 比例参数设置
		* @param[in]      kd: 微分参数设置
		* @retval         none
		*/
		
  while (1)
  {
		if(receive_flag=='s'){
			bujin(1,45,0);//启动
			HAL_UART_Transmit(&huart1,"1\r\n",3,0xff);
			receiveData[0] = '0';
			receive_flag='0';
		}else if(receive_flag=='g'){
			get(ref);
			HAL_UART_Transmit(&huart1,"1\r\n",3,0xff);
			receiveData[0]='0';
			receive_flag='0';    
		}else if(receive_flag=='p'){
			place(ref);
			HAL_UART_Transmit(&huart1,"1\r\n",3,0xff);
			receiveData[0]='0';
			receive_flag='0';
		}else if(receive_flag=='n'){
			MiAngleControlOn(ref);
			receiveData[0]='0';
			receive_flag='0';
			HAL_UART_Transmit(&huart1,"1\r\n",3,0xff);
		}else if(receive_flag=='f'){
			MiAngleControlOff(ref);
			receiveData[0]='0';
			receive_flag='0';
			HAL_UART_Transmit(&huart1,"1\r\n",3,0xff);
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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart,uint16_t Size){
	if(huart==&huart1){//第一位设为标志位
		receive_flag = receiveData[0];
		ref=atof(&receiveData[1]);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,receiveData,10);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
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
