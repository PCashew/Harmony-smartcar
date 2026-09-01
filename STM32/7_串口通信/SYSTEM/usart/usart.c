#include "sys.h"
#include "usart.h"	  

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 


#if EN_USART1_RX   //如果使能了接收

u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 USART_RX_STA=0;       //接收状态标记	  
u8  USART_RX_COUNT=0;    //接收计数
int CAR_buff[4];                                   //小车接收到数据    0:dir   1:motorA     2:motorB    3:angle
u8 uart_rec_flag=0;                                //串口帧标志
void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		{
			Res =USART_ReceiveData(USART1);	//读取接收到的数据		
			USART_RX_BUF[USART_RX_COUNT]=Res;

			if(USART_RX_BUF[0]==0xFC)        //寻找帧头
				USART_RX_COUNT++;
			else 
				USART_RX_COUNT=0;
			
			if(USART_RX_BUF[5]==0xFD)            //得到帧尾    一帧数据
			{
				USART_RX_COUNT=0;
				CAR_buff[0]=USART_RX_BUF[1];        //方向A
				CAR_buff[1]=USART_RX_BUF[2];        //电机A
				CAR_buff[2]=USART_RX_BUF[3];        //方向B
				CAR_buff[3]=USART_RX_BUF[4];        //电机B
				memset(USART_RX_BUF,0,6);
				uart_rec_flag=1;                       //串口帧标志		
			}

			USART_ClearFlag(USART1, USART_FLAG_RXNE);   //清除中断标志位
     } 
} 




/***调试用函数***/
//*****发送字符串****//
void put_string(USART_TypeDef* USARTx, char* str)
{
	int i = 0;
	//1.判断是否为'\0',是则发送完成，不是则一个一个字符发送
	while(str[i] != '\0')
	{
		USART_SendData(USARTx, str[i]);
		while ( USART_GetFlagStatus(USARTx, USART_FLAG_TXE )==RESET);
		i++;
	}
//	//3.发送完成换行，光标回到首位
//	USART_SendData(USARTx, '\r');
//	while ( USART_GetFlagStatus(USARTx, USART_FLAG_TXE )==RESET);
//	USART_SendData(USARTx, '\n');
//	while ( USART_GetFlagStatus(USARTx, USART_FLAG_TXE )==RESET);
	
}
 

//*******发送数据串************//
void put_HEX(USART_TypeDef* USARTx, u8* buf, u16 len)
{
	uint16_t i;
	
	for(i = 0;i < len; i++)          //写入发送缓存
	{
		USART_SendData(USARTx, buf[i]);
		while ( USART_GetFlagStatus(USARTx, USART_FLAG_TXE )==RESET);
	}
}

//*****发送数字******//
void put_shuzu(USART_TypeDef* USARTx,double num)
{
	u8 i;
	int sig;
	char num_char[8];
	long number = 0;
	
	if(num>0)sig=1;
	else if(num==0)sig=0;
	else if(num<0){sig=-1;num=-num;}
	
	number=(long)(num*1000);
	
	num_char[0]=(char)(number/1000000%10)+'0';
	num_char[1]=(char)(number/100000%10)+'0';
	num_char[2]=(char)(number/10000%10)+'0';  
	num_char[3]=(char)(number/1000%10)+'0';
	num_char[4]='.';
	num_char[5]=(char)(number/100%10)+'0';
	num_char[6]=(char)(number/10%10)+'0';
	num_char[7]=(char)(number%10)+'0';
	switch (sig)
	{	
		case 1 :  USART_SendData(USARTx, '+'); while ( USART_GetFlagStatus(USARTx, USART_FLAG_TXE )==RESET);break;
		case 0 :  USART_SendData(USARTx, ' '); while ( USART_GetFlagStatus(USARTx, USART_FLAG_TXE )==RESET);break;
		case -1 : USART_SendData(USARTx, '-'); while ( USART_GetFlagStatus(USARTx, USART_FLAG_TXE )==RESET);break;
			
	} 
 
	for(i=0;i<8;i++)
	{
		USART_SendData(USARTx, num_char[i]);
		while ( USART_GetFlagStatus(USARTx, USART_FLAG_TXE )==RESET);
	}
   USART_SendData(USARTx, '\r');
	 while ( USART_GetFlagStatus(USARTx, USART_FLAG_TXE )==RESET);
	USART_SendData(USARTx, '\n');
	 while ( USART_GetFlagStatus(USARTx, USART_FLAG_TXE )==RESET);
}



#endif	

