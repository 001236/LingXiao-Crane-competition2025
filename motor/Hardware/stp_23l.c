#include "usart.h"
#include "stp_23l.h"

char USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
uint16_t point1 ;
LidarPointTypedef Pack_Data[12];/* 雷达接收的数据储存在这个变量之中 */
LidarPointTypedef Pack_sum;     /* 输出结果储存 */
extern uint16_t receive_cnt;
extern uint8_t confidence;
extern uint16_t distance,noise,reftof;
extern uint32_t peak,intg;


void data_process(void)/*数据处理函数，完成一帧之后可进行数据处理*/
{
		/* 计算距离 */
		static u8 cnt = 0;
		u8 i;
		static u16 count = 0;
		static u32 sum = 0;
		static u32 distance_sum = 0;
		static u32 noise_sum = 0;
		static u32 peak_sum = 0;
		static u32 confidence_sum = 0;
		static u32 intg_sum = 0;
		static u32 reftof_sum = 0;
		for(i=0;i<12;i++)									/* 12个点取平均 */
		{
				if(Pack_Data[i].distance != 0)  /* 去除0的点 */
				{
						count++;
						distance_sum += Pack_Data[i].distance;
						noise_sum += Pack_Data[i].noise;
						peak_sum += Pack_Data[i].peak;
						confidence_sum += Pack_Data[i].confidence;
						intg_sum += Pack_Data[i].intg;
						reftof_sum += Pack_Data[i].reftof;
				}
		}
		if(count !=0)
		{
			distance = distance_sum/count;
			noise = noise_sum/count;
			peak = peak_sum/count;
			confidence = confidence_sum/count;
			intg = intg_sum/count;
			reftof = reftof_sum/count;
			count = 0;
			
			sum = 0;
			distance_sum = 0;
			noise_sum = 0;
			peak_sum = 0;
			confidence_sum = 0;
			intg_sum = 0;
			reftof_sum = 0;
		}
}
