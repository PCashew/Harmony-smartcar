#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  			7  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u8 USART_RX_STA;         		//接收状态标记	
extern int CAR_buff[4];         //小车接收到数据    0:dir   1:motorA     2:motorB    3:angle
extern u8 uart_rec_flag;     
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);

#endif


