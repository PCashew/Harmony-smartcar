#include "usart.h"

/* 支持printf重定向，避免使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE {
  int handle;
};

FILE __stdout;

void _sys_exit(int x) { (void)x; }

int fputc(int ch, FILE *stream) {
  (void)stream;
  while ((USART1->SR & USART_FLAG_TC) == 0)
    ;
  USART1->DR = (u8)ch;
  return ch;
}

#if EN_USART1_RX

static volatile u8 received_data;
static volatile u8 received_ready;

/**************************************************************************
函数功能：初始化USART1
入口参数：bound，串口波特率
返回值  ：无
硬件连接：PA9-TX，PA10-RX
**************************************************************************/
void uart_init(u32 bound) {
  GPIO_InitTypeDef gpio;
  USART_InitTypeDef usart;
  NVIC_InitTypeDef nvic;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA,
                         ENABLE);

  gpio.GPIO_Pin = GPIO_Pin_9;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &gpio);

  gpio.GPIO_Pin = GPIO_Pin_10;
  gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &gpio);

  nvic.NVIC_IRQChannel = USART1_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 3;
  nvic.NVIC_IRQChannelSubPriority = 3;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  USART_StructInit(&usart);
  usart.USART_BaudRate = bound;
  usart.USART_WordLength = USART_WordLength_8b;
  usart.USART_StopBits = USART_StopBits_1;
  usart.USART_Parity = USART_Parity_No;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &usart);

  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void) {
  u8 data;

  if (USART_GetITStatus(USART1, USART_IT_RXNE) == RESET)
    return;

  data = (u8)USART_ReceiveData(USART1);
  if (data == '\r' || data == '\n') // 忽略串口工具附加的换行符
    return;

  received_data = data;
  received_ready = 1;
}

/**************************************************************************
函数功能：非阻塞读取一个USART1指令字节
入口参数：data，接收字节的保存地址
返回值  ：1表示读取成功，0表示当前无数据或参数无效
**************************************************************************/
u8 UART_TryReadByte(u8 *data) {
  if (data == 0)
    return 0;

  /* 临界区内读取，防止中断在读取过程中覆盖状态 */
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
  if (received_ready == 0) {
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    return 0;
  }

  *data = received_data;
  received_ready = 0;
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  return 1;
}

#endif
