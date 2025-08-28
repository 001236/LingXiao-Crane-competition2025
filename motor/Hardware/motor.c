#include "motor.h"
/**
*@note：电机初始化函数
*/
__IO uint16_t target=0;
__IO uint16_t counter_now=0;
__IO float rpm=0;
uint16_t targetp=0;
double counter_posi=0.0;
extern uint16_t distance;
extern int current_wheel_state;
void motor_init(TIM_HandleTypeDef htim,TIM_HandleTypeDef htim_PWM,uint16_t TIM_CHANNELHanle ,motor_data *motor_)
{
motor_->MOTOR_DIR=0;
motor_->dutyfactor=0;
motor_->is_motor_enable=0;
motor_->actual_speed=0;
motor_->TIM_EncoderHandle=htim;
motor_->TIM_PWMHandle=htim_PWM;
motor_->TIM_CHANNELHanle=TIM_CHANNELHanle;
motor_->motor_Capture_Count=65536.0/2.0;
PID_Param_init_p(&(motor_->motor_pid_p));
PID_Param_init_v(&(motor_->motor_pid_v));
}
/**
*@note：电机使能函数
void set_motor_enable(motor_data *motor_)
{
  motor_->is_motor_enable=1;
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,1);
}
*/
/**
*@note：电机失能函数
*/
/**void set_motor_disable(motor_data *motor_)
{
  motor_->is_motor_enable=0;
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin,0);
}
*/

//电机开启
void motor_start(motor_data *motor_){
	HAL_TIM_PWM_Start(&htim1,motor_->TIM_CHANNELHanle);
	__HAL_TIM_SetCounter(&motor_->TIM_EncoderHandle,32768);
	HAL_TIM_Encoder_Start(&motor_->TIM_EncoderHandle,TIM_CHANNEL_ALL);
}
//电机关闭
void motor_stop(motor_data *motor_){
	HAL_TIM_PWM_Stop(&htim1,motor_->TIM_CHANNELHanle);
	//HAL_TIM_Base_Stop_IT(&htim1);
	//HAL_TIM_Encoder_Stop(&htim2,TIM_CHANNEL_ALL);
}
//电机转向
void set_motor_direction(motor_data *motor_,GPIO_TypeDef* GPIOx1,uint16_t GPIO_Pin1,GPIO_TypeDef* GPIOx2,uint16_t GPIO_Pin2){
	if(motor_->motor_pid_v.output>0){
      HAL_GPIO_WritePin(GPIOx1, GPIO_Pin1, 0); // 和设置的初始转向相同
      HAL_GPIO_WritePin(GPIOx2, GPIO_Pin2, 1);
		  __HAL_TIM_SET_COMPARE(&htim1, motor_->TIM_CHANNELHanle, motor_->motor_pid_v.output);//这里必须分开写，否则不能正确设定值。HAL库不支持多通道或运算调用
		}
		else if(motor_->motor_pid_v.output<0){
      HAL_GPIO_WritePin(GPIOx1, GPIO_Pin1, 1); // 和设置的初始转向相反
      HAL_GPIO_WritePin(GPIOx2, GPIO_Pin2, 0);
      __HAL_TIM_SET_COMPARE(&htim1,motor_->TIM_CHANNELHanle, -(motor_->motor_pid_v.output));
		}
}
//转速
void set_motor_speed(motor_data *motor_,float rpm){
	motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*rpm;
}
//圈数
void set_motor_posi(motor_data *motor_,double rotations){
	motor_->motor_pid_p.ref=rotations;
}
void set_pidp_ref(motor_data* motor,double ref){
	motor_init(motor->TIM_EncoderHandle,htim1,motor->TIM_CHANNELHanle, motor);
	set_motor_posi(motor,ref);
	motor_start(motor);
}
//pid控制
void motor_pid_control_long(motor_data *motor_){//依据位置环提供当前误差分析，调节速度环的目标值，速度环负责输出
	counter_now=__HAL_TIM_GetCounter(&motor_->TIM_EncoderHandle);
//	counter_posi=(counter_now-32768.0)/(459*4);
	motor_->motor_pid_p.fdb=distance;
	PID_Pos_Calc(&motor_->motor_pid_p);
	motor_->errorPercent=motor_->motor_pid_p.cur_error/motor_->motor_pid_p.ref;
	if(motor_->errorPercent>=0.9){//给予电机一个启动速度，避免启动时动力不够堵转
		motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*40;//考虑了设定位置目标值的正负，这就可以保证速度环输出方向的正确性
	}
	if(motor_->errorPercent<=0.9&&motor_->errorPercent>=0.3){//100等参数是使几个条件句联系起来，避免输出震荡而设置的
    motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*(100*(1-motor_->errorPercent)+30);
	}
	if(0<motor_->errorPercent<=0.3){
		motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*250*(motor_->errorPercent+0.1);//0.1是考虑了末端阻力，还需要根据实际情况调整
	}
	if(motor_->errorPercent<=0){//使电机停转，可能还会用到驱动的刹车模式，后续启动时还要调用电机启动函数
		motor_stop(motor_);
		HAL_UART_Transmit_DMA(&huart2,"1",10);
	}
	rpm=((counter_now-32768)*1000*60/10/4/459);
	motor_->motor_pid_v.fdb=rpm;
	PID_Calc(&motor_->motor_pid_v);
	__HAL_TIM_SET_COUNTER(&motor_->TIM_EncoderHandle,32768);
}
//void motor_pid_control(motor_data *motor_){//依据位置环提供当前误差分析，调节速度环的目标值，速度环负责输出
//	counter_now=__HAL_TIM_GetCounter(&motor_->TIM_EncoderHandle);
////	counter_posi=(counter_now-32768.0)/(459*4);
//	motor_->motor_pid_p.fdb=distance;
//	PID_Pos_Calc(&motor_->motor_pid_p);
//	if(motor_->motor_pid_p.cur_error>=50){
//		motor_->motor_pid_v.ref=500;
//	}
//	else if(motor_->motor_pid_p.cur_error>=10){
//		motor_->motor_pid_v.ref=60;
//	}
//	else if(abs(motor_->motor_pid_p.cur_error)<10){
//		motor_->motor_pid_v.ref=motor_->motor_pid_p.cur_error*2;
//	}
//	else if(motor_->motor_pid_v.fdb<=20&&abs(motor_->motor_pid_p.cur_error)<=0.01){
//		//motor_stop(motor_);
//		HAL_UART_Transmit_DMA(&huart6,"1",10);
//	}
////	motor_->errorPercent=motor_->motor_pid_p.cur_error/motor_->motor_pid_p.ref;
////	if(motor_->errorPercent>=0.9){//给予电机一个启动速度，避免启动时动力不够堵转
////		motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*15;//考虑了设定位置目标值的正负，这就可以保证速度环输出方向的正确性
////	}
////	if(motor_->errorPercent<=0.9&&motor_->errorPercent>=0.3){//100等参数是使几个条件句联系起来，避免输出震荡而设置的
////    motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*(50*(1-motor_->errorPercent)+10);
////	}
////	if(0<motor_->errorPercent<=0.3){
////		motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*45/0.4*(motor_->errorPercent+0.1);//0.1是考虑了末端阻力，还需要根据实际情况调整
////	}
////	if(motor_->errorPercent<=0){//使电机停转，可能还会用到驱动的刹车模式，后续启动时还要调用电机启动函数
////		motor_stop(motor_);
////		HAL_UART_Transmit_DMA(&huart2,"1",10);
////	}
//	rpm=((counter_now-32768)*1000*60/10/4/459);
//	motor_->motor_pid_v.fdb=rpm;
//	PID_Calc(&motor_->motor_pid_v);
//	__HAL_TIM_SET_COUNTER(&motor_->TIM_EncoderHandle,32768);
//}
void motor_pid_control_short(motor_data *motor_){//依据位置环提供当前误差分析，调节速度环的目标值，速度环负责输出
	counter_now=__HAL_TIM_GetCounter(&motor_->TIM_EncoderHandle);
	counter_posi=(counter_now-32768.0)/(459*4);
	motor_->motor_pid_p.fdb+=counter_posi;
	PID_Pos_Calc(&motor_->motor_pid_p);
	motor_->errorPercent=motor_->motor_pid_p.cur_error/motor_->motor_pid_p.ref;
	if(motor_->errorPercent>=0.9){//给予电机一个启动速度，避免启动时动力不够堵转
		motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*15;//考虑了设定位置目标值的正负，这就可以保证速度环输出方向的正确性
	}
	if(motor_->errorPercent<=0.9&&motor_->errorPercent>=0.3){//100等参数是使几个条件句联系起来，避免输出震荡而设置的
    motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*(50*(1-motor_->errorPercent)+10);
	}
	if(0<motor_->errorPercent<=0.3){
		motor_->motor_pid_v.ref=(motor_->motor_pid_p.ref>0?1:-1)*45/0.4*(motor_->errorPercent+0.1);//0.1是考虑了末端阻力，还需要根据实际情况调整
	}
	if(motor_->errorPercent<=0){//使电机停转，可能还会用到驱动的刹车模式，后续启动时还要调用电机启动函数
		motor_stop(motor_);
		HAL_UART_Transmit_DMA(&huart2,"1",10);
	}
	rpm=((counter_now-32768)*1000*60/10/4/459);
	motor_->motor_pid_v.fdb=rpm;
	PID_Calc(&motor_->motor_pid_v);
	__HAL_TIM_SET_COUNTER(&motor_->TIM_EncoderHandle,32768);
}
