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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "receive_from_pi.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 机器人运行的主状�?
typedef enum {
	STATE_BEFORE_START,
    STATE_INIT,
    STATE_WAIT_RPI_DATA,
    STATE_PROCESS_TASK_DATA,
    STATE_START_ROBOT_POSE,
    STATE_WAIT_START_POSE_ACK,

    /* --- 以�?�列”为单位的循�? --- */
    STATE_GET_NEXT_COLUMN,          // 获取下一个要处理的货架列
    STATE_MOVE_TO_SHELF_COLUMN,     // 移动到目标货架列
    STATE_WAIT_MOVE_TO_SHELF_ACK,
    
    STATE_EXECUTE_DOUBLE_PICK,      // 执行“双取�?�动�?
    STATE_WAIT_DOUBLE_PICK_ACK,
	  STATE_OUTOFTIME,

    /* --- 处理第一个箱�? (上层箱子) --- */
    STATE_PLAN_TARGET_A_MOVE,       // 规划去往A目标(上层箱子)的移�?
    STATE_EXECUTE_MAJOR_MOVE_A,
    STATE_WAIT_MAJOR_MOVE_A_ACK,
//    STATE_EXECUTE_FINE_TUNE_MOVE_A,
//    STATE_WAIT_FINE_TUNE_MOVE_A_ACK,
    STATE_PLACE_BOX_A,              // 放置上层箱子
    STATE_WAIT_PLACE_BOX_A_ACK,

    /* --- 内部转接 --- */
//    STATE_EXECUTE_ARM_REGRAB,       // 执行机械臂从前爪的�?�转接�?�动�?
//    STATE_WAIT_ARM_REGRAB_ACK,

    /* --- 处理第二个箱�? (下层箱子) --- */
    STATE_PLAN_TARGET_B_MOVE,       // 规划去往B目标(下层箱子)的移�?
    STATE_EXECUTE_MAJOR_MOVE_B,
    STATE_WAIT_MAJOR_MOVE_B_ACK,
//    STATE_EXECUTE_FINE_TUNE_MOVE_B,
//    STATE_WAIT_FINE_TUNE_MOVE_B_ACK,
    STATE_PLACE_BOX_B,              // 放置下层箱子
    STATE_WAIT_PLACE_BOX_B_ACK,

    STATE_ALL_TASKS_COMPLETED,
		STATE_WAIT_OFF,
		STATE_SWITCH_OFF,
    STATE_ERROR
} RobotState_t;

typedef enum {
    ROBOT_POS_UNKNOWN,         // 未知位置
    ROBOT_POS_SHELF,           // 在货架区
    ROBOT_POS_SIDE_AREA,       // 在侧方放置区 (对应1, 6号纸�?)
    ROBOT_POS_FRONT_AREA,      // 在前方放置区 (对应2, 3, 4, 5号纸�?)
} RobotPosition_t;

// 封装�?有任务信息的结构�?
typedef struct {
	  int shelf_to_target[7];         // 货架格上箱子编号�?1-6�?
    int shelf_slot_to_area_idx[7];  // 货架�?(索引1-6) -> 目标区域索引(�?0-5, 对应a-f)

    // 孤儿箱子信息
    int orphan_box_id;              // “孤儿箱子�?�的编号 (1-6)
    int orphan_box_initial_shelf;   // "孤儿箱子"�?初所在的货架�?(1-3)

    // 执行顺序规划
    int column_run_order[3];        // 列的执行顺序
    int box_run_order[6];           // 箱子的执行顺�? (根据列顺序生�?)

    // 运行时状�?
    int current_column_idx;         // 当前执行�? column_run_order 的哪个索�?
	  int current_box_idx;
    int last_placed_target_id;      // 上一个放置的箱子对应的纸垛编�?
    
    // 用于临时存储当前正在处理的两个箱子的信息
    int top_box_id;
    int bottom_box_id;
} TaskInfo_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// --- 全局变量 ---
uint8_t receiveData[4][10];
volatile int wheel_ack = 0;    // 轨道电机 ACK

volatile int jixiebi_ack=0;
volatile int dajiang_ack = 0;  // 水平滑台 ACK (你的代码中是huart6)
volatile RobotPosition_t current_robot_position = ROBOT_POS_UNKNOWN;
RobotState_t current_state = STATE_INIT; // 当前状�?�机状�??
TaskInfo_t task_info;                    // 当前任务信息
RecognitionResult_t recognition_data;    // 从树莓派接收的原始数�?
int ifreturn;
int posi=-1;
double start_time , current_time,duration;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 根据货架格号(1-6)返回其所在的列号(1-3)
int get_column_from_shelf_slot(int slot) {
    if (slot == 1 || slot == 4) return 1;
    if (slot == 2 || slot == 5) return 2;
    if (slot == 3 || slot == 6) return 3;
    return 0; // 错误
}

/**
 * @brief  【最终版】处理数据，规划�?有任务和映射
 */
void process_and_plan_task(RecognitionResult_t* rec_data, TaskInfo_t* task) {
    // 1. 反向映射，几号纸垛放置在哪个位置
    int box_id_to_area_idx[7] = {-1, -1, -1, -1, -1, -1, -1};
    for (int j = 0; j < 6; j++) {
        if (rec_data->area_positions[j] != 0) {
            box_id_to_area_idx[rec_data->area_positions[j]] = j;
        }
    }

    // 2. 找到孤儿箱子ID
		
    task->orphan_box_id = 0;
    for (int k = 1; k <= 6; k++) {
        if (box_id_to_area_idx[k] == -1) {
            task->orphan_box_id = k;
            break;
        }
    }

    // 3. 构建 货架�? -> 纸垛ID �? 货架�? -> 区域索引 的双重映�?
    for (int i = 0; i < 6; i++) {
        int shelf_slot = i + 1;
        int box_id = rec_data->shelf_positions[i];
        task->shelf_to_target[shelf_slot] = box_id; 
			task->shelf_slot_to_area_idx[shelf_slot] = box_id_to_area_idx[box_id];//box_id_to_area_idx[box_id]的�?�为0-5，表示纸垛区的a-f，shelf_id_to_area_idx[shelf_slot]中，shelf_slot为货架格�?1-6，故该映射为货架格号到纸垛位置的映射
        if (box_id == task->orphan_box_id) {
            task->orphan_box_initial_shelf = (shelf_slot-1)%3+1;
        }
    }
    
    // 4. 【按列搬运�?�规划执行顺�?
    int orphan_column = get_column_from_shelf_slot(task->orphan_box_initial_shelf);
    int column_order_idx = 0;
    for (int col = 1; col <= 3; col++) {//列号1-3
        if (col != orphan_column) task->column_run_order[column_order_idx++] = col;
    }
    task->column_run_order[column_order_idx] = orphan_column;

    // 5. 根据列顺序生成最终的箱子搬运顺序 (box_run_order)
    int run_order_idx = 0;
    for (int i = 0; i < 3; i++) {
        int current_col = task->column_run_order[i];
        int top_shelf_slot = current_col;
        int bottom_shelf_slot = current_col + 3;
        task->box_run_order[run_order_idx++] = rec_data->shelf_positions[top_shelf_slot - 1];
        task->box_run_order[run_order_idx++] = rec_data->shelf_positions[bottom_shelf_slot - 1];
    }
    
    // 6. 初始化任务指�?
    task->current_column_idx = 0;
    task->last_placed_target_id = 0;
}

void sendCommand_MoveToStart(){//huart1-wheel,huart2-front,huart3-jixiebi
	HAL_UART_Transmit_DMA(&huart3,"s",10);
	HAL_Delay(500);
	HAL_UART_Transmit_DMA(&huart2,"s",10);
}

void sendCommand_MoveToShelf(int column_id,int target_id) {
    // 根据 box_id 确定它在哪个货架格，然后计算水平滑台�?要移动的距离
	int last_target_id=0;
	if(task_info.current_column_idx!=0){
     last_target_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx-1]+3];
		 if(last_target_id==-1){
			 last_target_id=task_info.last_placed_target_id;
		 }
	}
	HAL_Delay(10);
	HAL_UART_Transmit_DMA(&huart2,"v3",10);
	HAL_Delay(10);
	if(column_id==1){
		HAL_UART_Transmit_DMA(&huart6,"A50",10);
	}else if(column_id==2){
		HAL_UART_Transmit_DMA(&huart6,"A0",5);
	}else if(column_id==3){
		HAL_UART_Transmit_DMA(&huart6,"A-52",10);
	}
	if(task_info.current_column_idx==0||last_target_id == 0){
		HAL_UART_Transmit_DMA(&huart3,"f0",10);
	}else{
		HAL_UART_Transmit_DMA(&huart3,"f2",10);
	}
	if(task_info.current_column_idx==0&&(task_info.column_run_order[0]==1||task_info.column_run_order[0]==3)){
		HAL_Delay(800);
	}else if(task_info.current_column_idx==0&&task_info.column_run_order[0]==2){
		HAL_Delay(500);
	}else if((last_target_id == 0||last_target_id == 1)&&task_info.column_run_order[task_info.current_column_idx]==1){
		HAL_Delay(1500);
	}else if((last_target_id == 4||last_target_id == 5)&&task_info.column_run_order[task_info.current_column_idx]==3){
		HAL_Delay(1500);
	}else {
		HAL_Delay(500);
	}
	HAL_UART_Transmit_DMA(&huart2,"h-2",10);
	HAL_UART_Transmit_DMA(&huart1,"a8.5",10);
	
}

void sendCommand_MoveTargetToTarget(RobotPosition_t targetPosition,int target_id){
	ifreturn=(posi>=1&&posi<=3&&target_id>=1&&target_id<=2)||(posi<=4&&posi>=3&&target_id>=3&&target_id<=4);
	if(current_robot_position==targetPosition&&ifreturn==1)
		return;
	if(targetPosition==ROBOT_POS_SIDE_AREA&&target_id==0){
		HAL_UART_Transmit_DMA(&huart1,"a269",5);
	}
	else if(targetPosition==ROBOT_POS_FRONT_AREA&&target_id>=1&&target_id<=3){
		HAL_UART_Transmit_DMA(&huart1,"a286.5",5);
	}else if(targetPosition==ROBOT_POS_FRONT_AREA&&target_id==4){
		HAL_UART_Transmit_DMA(&huart1,"a283",5);
	}else if(targetPosition==ROBOT_POS_SIDE_AREA&&target_id==5){
		HAL_UART_Transmit_DMA(&huart1,"a268",5);
	}
}

void sendCommand_ArmRegrab(){
	int last_target_id=0;
  last_target_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];
	if(last_target_id == 0){
		HAL_UART_Transmit_DMA(&huart3,"f0",10);
	}else {
		HAL_UART_Transmit_DMA(&huart3,"f2",10);
	}
	while(jixiebi_ack==0);
	HAL_UART_Transmit_DMA(&huart3,"g26",5);
}

void sendCommand_PickBox(){
	HAL_UART_Transmit_DMA(&huart3,"g11",10);//先机械臂夹起�?
	HAL_Delay(50);
	HAL_UART_Transmit_DMA(&huart2,"h0",10);//收缩夹住箱子
	HAL_Delay(1000);
	HAL_UART_Transmit_DMA(&huart2,"v10",10);//提升前爪
}

void sendCommand_PlaceBox(int box_id, TaskInfo_t* task,int serial_number) {
	 if(serial_number==0){
        HAL_UART_Transmit_DMA(&huart3,"n93",5);
		    while(jixiebi_ack==0);
		 if (box_id == task->orphan_box_id){
				HAL_UART_Transmit_DMA(&huart3,"p21",10);//放置
			}else {
				HAL_UART_Transmit_DMA(&huart3,"p33",10);//放置
			}
	 }else if(serial_number >= 1 && serial_number <= 4){
       	HAL_UART_Transmit_DMA(&huart3,"n-180",10);
       	while(jixiebi_ack==0);
		  if (box_id == task->orphan_box_id){
				HAL_UART_Transmit_DMA(&huart3,"p21",10);//放置
			}else {
				HAL_UART_Transmit_DMA(&huart3,"p33",10);//放置
			}
       }else if(serial_number==5){
       	HAL_UART_Transmit_DMA(&huart3,"n-91",5);
       	while(jixiebi_ack==0);
				if (box_id == task->orphan_box_id){
				HAL_UART_Transmit_DMA(&huart3,"p21",10);//放置
			}else {
				HAL_UART_Transmit_DMA(&huart3,"p33",10);//放置
			}
	 }
}

void start_clock(){
	__HAL_TIM_SetCounter(&htim2,0);
	start_time=HAL_GetTick();
	HAL_TIM_Base_Start(&htim2);
	
}
void calculateDuration(){
	current_time=HAL_GetTick();
	duration=(current_time-start_time)/1000;
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
HAL_UART_Receive_IT(&huart1,receiveData[0],3);
HAL_UART_Receive_IT(&huart2,receiveData[1],3);
HAL_UART_Receive_IT(&huart3,receiveData[2],3);
HAL_UART_Receive_IT(&huart6,receiveData[3],3);
HAL_UART_Transmit_DMA(&huart2,"h0",5);
//recognition_data.shelf_positions[0]=3;
//recognition_data.shelf_positions[1]=4;
//recognition_data.shelf_positions[2]=2;
//recognition_data.shelf_positions[3]=5;
//recognition_data.shelf_positions[4]=6;
//recognition_data.shelf_positions[5]=1;
//recognition_data.area_positions[0]=1;
//recognition_data.area_positions[1]=2;
//recognition_data.area_positions[2]=0;
//recognition_data.area_positions[3]=4;
//recognition_data.area_positions[4]=5;
//recognition_data.area_positions[5]=6;
// 初始化接收模块，告诉它使用UART5
  rpi_init_polling(&huart5);
	HAL_TIM_Base_Init(&htim2);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1&&current_state==STATE_BEFORE_START){
			current_state=STATE_START_ROBOT_POSE;//修改
		}
		switch (current_state)
        {
					case STATE_BEFORE_START:{
							break;
			      }
            case STATE_INIT:{
							// 初始化任务变�?
                memset(&task_info, 0, sizeof(TaskInfo_t));
                wheel_ack = dajiang_ack = jixiebi_ack = 0;
                // 直接进入下一个状�?
                current_state = STATE_WAIT_RPI_DATA;//�?
                break;
						}
                
            case STATE_WAIT_RPI_DATA:{
							// 调用阻塞函数获取数据，状态机会在这里暂停直到收到数据
                // 如果有非阻塞函数，会更理想，但目前这样也可以工作
                rpi_request_and_receive_configurable_polling(
                                &recognition_data,
                                "REQUEST\n", // 主动发�?�这个信号给树莓�?
                                true,    // 启用修复
                                5,       // 尝试5次后修复货架
                                false,    // 禁用超时
                                30000);  // 30秒超�?
                current_state = STATE_PROCESS_TASK_DATA;
                break;
						}
                
            case STATE_PROCESS_TASK_DATA:{
							// --- 这是核心规划逻辑 ---
                process_and_plan_task(&recognition_data, &task_info);
                current_state = STATE_BEFORE_START;
							  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
                break;
						}
                
            case STATE_START_ROBOT_POSE:{
							// 发�?�指令，让机器人移动到货架区的初始位�?
                sendCommand_MoveToStart(); //启动至货�?
							  wheel_ack=1;
                current_state = STATE_WAIT_START_POSE_ACK;
                break;
						}
                
            case STATE_WAIT_START_POSE_ACK:{
							if(wheel_ack == 1){ 
                    wheel_ack = 0; // 重置ACK
                    current_state = STATE_GET_NEXT_COLUMN;
                }
                // 此处可以添加超时逻辑
                break;
						}
                

            case STATE_GET_NEXT_COLUMN:{
							if (task_info.current_column_idx >= 3) { // 承朿6个箱子都处理完了
                    current_state = STATE_ALL_TASKS_COMPLETED;
                } else {
                    current_state = STATE_MOVE_TO_SHELF_COLUMN;
                }
                break;
						}
                

            case STATE_MOVE_TO_SHELF_COLUMN:
                {
									int is_orphan = (task_info.bottom_box_id == task_info.orphan_box_id);
                  int target_id = is_orphan ? task_info.last_placed_target_id : task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
                  int shelf_column_id = task_info.column_run_order[task_info.current_column_idx];
									task_info.bottom_box_id=recognition_data.shelf_positions[shelf_column_id+2];
									task_info.top_box_id=recognition_data.shelf_positions[shelf_column_id-1];
								  wheel_ack = dajiang_ack=0;
									
									
										sendCommand_MoveToShelf(shelf_column_id,target_id);
									
                    current_state = STATE_WAIT_MOVE_TO_SHELF_ACK;
                }
                break;

            case STATE_WAIT_MOVE_TO_SHELF_ACK:{
							if ((dajiang_ack >= 1)&&(wheel_ack>=1)) { 
                    wheel_ack = dajiang_ack = jixiebi_ack = 0;
                    current_state = STATE_EXECUTE_DOUBLE_PICK;
									  current_robot_position=ROBOT_POS_SHELF;
                }
                break;
						}
                

            case STATE_EXECUTE_DOUBLE_PICK:{
							sendCommand_PickBox();
                // 根据取货动作序列，设置期望的ACK数量
							start_clock();
                current_state = STATE_WAIT_DOUBLE_PICK_ACK;
                break;
						}
                

            case STATE_WAIT_DOUBLE_PICK_ACK:{
							calculateDuration();
							if (jixiebi_ack >= 1) { // 棿查是否收到承有ACK
								jixiebi_ack = 0;
                    current_state = STATE_PLAN_TARGET_A_MOVE;
								    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
                }else if(duration>=3){
							  	  HAL_TIM_Base_Stop(&htim2);
							  	  TIM2->CNT = 0;
							  	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
							  	  current_state=STATE_OUTOFTIME;
							  }
							
                break;
						}
						case STATE_OUTOFTIME:{
							jixiebi_ack = 1;
							current_state=STATE_WAIT_DOUBLE_PICK_ACK;
							start_clock();
							break;
						}
                
								
            case STATE_PLAN_TARGET_A_MOVE:{
                int is_orphan = (task_info.top_box_id == task_info.orphan_box_id);
                int target_id = is_orphan ? task_info.last_placed_target_id : task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];

                RobotPosition_t required_pos = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;

                if (current_robot_position != required_pos) {
                    // �?要进行大的跨区域移动
                    current_state = STATE_EXECUTE_MAJOR_MOVE_A;
                } else {
                    current_state = STATE_PLACE_BOX_A;
                }
            }
            break;

        case STATE_EXECUTE_MAJOR_MOVE_A:{ // 执行大的跨区域移�?
                int is_orphan = (task_info.top_box_id == task_info.orphan_box_id);
                int target_id = is_orphan ? task_info.last_placed_target_id : task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];
                RobotPosition_t required_pos = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;
					//执行函数
					      wheel_ack = dajiang_ack = 0;
					
                sendCommand_MoveTargetToTarget(required_pos,target_id);
					      HAL_UART_Transmit_DMA(&huart2,"v20",5);
					      HAL_Delay(500);
			         	 if(target_id==0){
		            	HAL_UART_Transmit_DMA(&huart6,"A-45",5);
		            }else if(target_id==1){
		            	HAL_UART_Transmit_DMA(&huart6,"A-68",5);
		            }else if(target_id==2){
		            	HAL_UART_Transmit_DMA(&huart6,"A-23",5);
		            }else if(target_id==3){
		            	HAL_UART_Transmit_DMA(&huart6,"A23",5);
		            }else if(target_id==4){
		            	HAL_UART_Transmit_DMA(&huart6,"A68",5);
		            }else if(target_id==5){
		            	HAL_UART_Transmit_DMA(&huart6,"A45",5);
		            }
					      
                current_state = STATE_WAIT_MAJOR_MOVE_A_ACK;
            }
            break;

        case STATE_WAIT_MAJOR_MOVE_A_ACK:{
					if (wheel_ack >= 1 && dajiang_ack >= 1) { // 等待轨道电机(huart1)的确�?
                wheel_ack = dajiang_ack = 0;

                // 【重要㿑移动完成后，更新当前位置状�??
                int is_orphan = (task_info.top_box_id == task_info.orphan_box_id);
                int target_id = is_orphan ? task_info.last_placed_target_id : task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];
                current_robot_position = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;
                current_state = STATE_PLACE_BOX_A;
            }
            break;
				} // 等待大的跨区域移动完�?
            
				
            case STATE_PLACE_BOX_A:{
                    int box_id_to_place = task_info.top_box_id;
									  int area_idx_to_go;
									  if (box_id_to_place == task_info.orphan_box_id) {
                    // 如果是孤儿箱子，我们要找到它叠放的目标箱�?(last_placed_target_id)�?在的区域
                    // 这需要我们反向查找一�? (或�?�在规划时也存一下这个映�?)
                    // 为了�?单，我们先假设�?�能找到
                    int final_target_id = task_info.last_placed_target_id;
                    area_idx_to_go=final_target_id;
                } else {
                    // 对于普�?�箱子，直接查找它自己应该去的区域索�?
                    for(int i=1; i<=6; i++){
                        if(task_info.shelf_to_target[i] == box_id_to_place){
                             area_idx_to_go = task_info.shelf_slot_to_area_idx[i];
                             break;
                        }
                    }
                }   
								    wheel_ack = dajiang_ack = jixiebi_ack = 0;
								//执行函数
                    sendCommand_PlaceBox(box_id_to_place, &task_info , area_idx_to_go);
								
                    current_state = STATE_WAIT_PLACE_BOX_A_ACK;
                }
                break;

            case STATE_WAIT_PLACE_BOX_A_ACK:{
							int area_idx_to_go=task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];
							if(task_info.orphan_box_id==task_info.top_box_id){
							  area_idx_to_go=task_info.last_placed_target_id;
						  }
							if((area_idx_to_go>=1&&area_idx_to_go<=4)&&jixiebi_ack >= 2){
                    jixiebi_ack = 0;
                    
                    // 更新上一个放置的ID，为孤儿箱子做准�?
                    int placed_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]];
									  int placed_box_id = task_info.top_box_id;
                    if (placed_box_id != task_info.orphan_box_id) {
                        task_info.last_placed_target_id = placed_id;
                    }

                    current_state = STATE_PLAN_TARGET_B_MOVE;
							}else if((area_idx_to_go==0||area_idx_to_go==5)&&jixiebi_ack >=2){
								wheel_ack = jixiebi_ack = 0;
								current_state = STATE_PLAN_TARGET_B_MOVE;
							}
							
                break;
						}
                 
							// ----- 放置下层箱子 (B) -----
        case STATE_PLAN_TARGET_B_MOVE:{
                int is_orphan = (task_info.bottom_box_id == task_info.orphan_box_id);
                int target_id = is_orphan ? task_info.last_placed_target_id : task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
                RobotPosition_t required_pos = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;
                if(current_robot_position != required_pos) current_state = STATE_EXECUTE_MAJOR_MOVE_B;
                else current_state = STATE_PLACE_BOX_B;
					sendCommand_ArmRegrab();
						
							if(target_id==0){
		            	HAL_UART_Transmit_DMA(&huart6,"A-45",5);
		            }else if(target_id==1){
		            	HAL_UART_Transmit_DMA(&huart6,"A-68",5);
		            }else if(target_id==2){
		            	HAL_UART_Transmit_DMA(&huart6,"A-23",5);
		            }else if(target_id==3){
		            	HAL_UART_Transmit_DMA(&huart6,"A23",5);
		            }else if(target_id==4){
		            	HAL_UART_Transmit_DMA(&huart6,"A68",5);
		            }else if(target_id==5){
		            	HAL_UART_Transmit_DMA(&huart6,"A45",5);
		            }
								
            }
            break;
        case STATE_EXECUTE_MAJOR_MOVE_B:{
                int is_orphan = (task_info.bottom_box_id == task_info.orphan_box_id);
                int target_id = is_orphan ? task_info.last_placed_target_id : task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
                RobotPosition_t required_pos = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;
                //执行函数
					      
					      sendCommand_MoveTargetToTarget(required_pos,target_id);
					      
					      wheel_ack=0;
                current_state = STATE_WAIT_MAJOR_MOVE_B_ACK;
            }
            break;
        case STATE_WAIT_MAJOR_MOVE_B_ACK:{
					if(wheel_ack >= 1 && dajiang_ack >= 1){
                wheel_ack = dajiang_ack = 0;
                int is_orphan = (task_info.bottom_box_id == task_info.orphan_box_id);
                int target_id = is_orphan ? task_info.last_placed_target_id : task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
                current_robot_position = (target_id == 0 || target_id == 5) ? ROBOT_POS_SIDE_AREA : ROBOT_POS_FRONT_AREA;
                current_state = STATE_PLACE_BOX_B;
            }
            break;
				}
            
						case STATE_PLACE_BOX_B:{
							int box_id_to_place = task_info.bottom_box_id;
							int area_idx_to_go;
							int is_orphan = (task_info.bottom_box_id == task_info.orphan_box_id);
							int target_id = is_orphan ? task_info.last_placed_target_id : task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
						  if (box_id_to_place == task_info.orphan_box_id) {
                    // 如果是孤儿箱子，我们要找到它叠放的目标箱�?(last_placed_target_id)承在的区�?
                    // 这需要我们反向查找一�? (或迅在规划时也存�?下这个映�?)
                    // 为了箿单，我们先假设濻能找到
								if(task_info.last_placed_target_id==task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]]){
									area_idx_to_go = task_info.last_placed_target_id;
								}else if(task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]]==0){
									if(recognition_data.area_positions[1]==0){
										target_id=2;
									}else if(recognition_data.area_positions[2]==0){
										target_id=1;
									}else {
										target_id=2;
									}
								}else if(task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]]==5){
									if(recognition_data.area_positions[4]==0){
										target_id=3;
									}else if(recognition_data.area_positions[3]==0){
										target_id=4;
									}else {
										target_id=3;
									}
								}
                    
                    
                } else {
                    // 对于普鿚箱子，直接查找它自己应该去的区域索弿
                    for(int i=1; i<=6; i++){
                        if(task_info.shelf_to_target[i] == box_id_to_place){
                             area_idx_to_go = task_info.shelf_slot_to_area_idx[i];
                             break;
                        }
                    }
                }   
								    wheel_ack = dajiang_ack = 0;
								//执行函数
								if(jixiebi_ack >= 2){
									  jixiebi_ack = 0;
									  sendCommand_PlaceBox(box_id_to_place, &task_info , area_idx_to_go);
								    
                    current_state = STATE_WAIT_PLACE_BOX_B_ACK;
								}
                    
                }
                break;
						
           case STATE_WAIT_PLACE_BOX_B_ACK:{
						 int area_idx_to_go=task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
						 if(task_info.orphan_box_id==task_info.bottom_box_id){
							 area_idx_to_go=task_info.last_placed_target_id;
						 }
						 if((area_idx_to_go>=1&&area_idx_to_go<=4)&&jixiebi_ack>=2){
                    wheel_ack = jixiebi_ack = 0;
                    
                    // 更新上一个放置的ID，为孤儿箱子做准�?
                    int placed_id = task_info.shelf_slot_to_area_idx[task_info.column_run_order[task_info.current_column_idx]+3];
									  int placed_box_id=task_info.bottom_box_id;
                    if (placed_box_id != task_info.orphan_box_id) {
                        task_info.last_placed_target_id = placed_id;
                    }
                    task_info.current_column_idx++;
                    current_state = STATE_GET_NEXT_COLUMN;
							}else if((area_idx_to_go==0||area_idx_to_go==5)&&jixiebi_ack>=2){
								wheel_ack = jixiebi_ack = 0;
								task_info.current_column_idx++;
								current_state = STATE_GET_NEXT_COLUMN;
							}
                break;
					 }
             
            case STATE_ALL_TASKS_COMPLETED:{
							// 可以在这里闪灯或发鿁完成信叿
                // 然后可以回到空闲状濁或停�?
						  HAL_UART_Transmit_DMA(&huart1,"a50",5);//移动到中心线左侧停止
							HAL_UART_Transmit_DMA(&huart6,"A0",5);
							HAL_UART_Transmit_DMA(&huart2,"v0",5);
							current_state=STATE_WAIT_OFF;
//                while(1) {
//                    // Mission Complete
//                }
                break;
						}
            case STATE_WAIT_OFF:{
							
							if(wheel_ack>=1){
								current_state=STATE_SWITCH_OFF;
							}
							break;
							
						}

						case STATE_SWITCH_OFF:{
							return 0;
						}

            case STATE_ERROR:{
							// 停止承有电机，闪烁错误LED
                while(1) {
                    // Error Halt
                }
                break;
						}
                
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==USART1){
		if(receiveData[0][0]=='1'&&receiveData[0][1]=='\r'){
			wheel_ack++;
			receiveData[0][0]='0';
			
		}HAL_UART_Receive_IT(&huart1,receiveData[0],3);
	}
		if(huart->Instance==USART2){
			if(receiveData[1][0]=='1'&&receiveData[1][1]=='\r'){
			receiveData[1][0]='0';
				
		}HAL_UART_Receive_IT(&huart2,receiveData[1],3);
		}
		if(huart->Instance==USART3){
			if(receiveData[2][0]=='1'&&receiveData[2][1]=='\r'){
			jixiebi_ack++;
			receiveData[2][0]='0';
				
		}HAL_UART_Receive_IT(&huart3,receiveData[2],3);
		}
		if(huart->Instance==USART6){
			if(receiveData[3][0]=='1'&&receiveData[3][1]=='\r'){
			dajiang_ack++;
			receiveData[3][0]='0';
				
		}HAL_UART_Receive_IT(&huart6,receiveData[3],3);
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
