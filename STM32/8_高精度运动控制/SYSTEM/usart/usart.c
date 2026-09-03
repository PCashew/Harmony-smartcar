#include "sys.h"
#include "usart.h"

#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;

void _sys_exit(int x)
{
    (void)x;
}

int fputc(int ch, FILE *f)
{
    (void)f;
    while ((USART1->SR & USART_FLAG_TC) == 0) { }
    USART1->DR = (u8)ch;
    return ch;
}

static volatile u8 rx_frame[PROTOCOL_FRAME_SIZE];
static volatile u8 ready_frame[PROTOCOL_FRAME_SIZE];
static volatile u8 rx_index = 0;
static volatile u8 frame_ready = 0;

static u8 Protocol_Checksum(const u8 *data)
{
    u8 i;
    u8 value = 0;
    for (i = 0; i < 8; ++i) {
        value ^= data[i];
    }
    return value;
}

void uart_init(u32 bound)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);

    nvic.NVIC_IRQChannel = USART1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    uart.USART_BaudRate = bound;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &uart);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

void USART1_IRQHandler(void)
{
    u8 value;
    u8 i;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == RESET) {
        return;
    }

    value = (u8)USART_ReceiveData(USART1);
    if (rx_index == 0 && value != PROTOCOL_HEAD_0) {
        return;
    }
    if (rx_index == 1 && value != PROTOCOL_HEAD_1) {
        rx_index = (value == PROTOCOL_HEAD_0) ? 1U : 0U;
        rx_frame[0] = value;
        return;
    }

    rx_frame[rx_index++] = value;
    if (rx_index < PROTOCOL_FRAME_SIZE) {
        return;
    }

    rx_index = 0;
    if (rx_frame[9] != PROTOCOL_TAIL || rx_frame[8] != Protocol_Checksum((const u8 *)rx_frame)) {
        return;
    }

    if (!frame_ready) {
        for (i = 0; i < PROTOCOL_FRAME_SIZE; ++i) {
            ready_frame[i] = rx_frame[i];
        }
        frame_ready = 1;
    }
}

u8 Protocol_GetFrame(u8 *command, u8 *seq, s16 *param1, s16 *param2)
{
    u8 data[PROTOCOL_FRAME_SIZE];
    u8 i;

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    if (!frame_ready) {
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        return 0;
    }
    for (i = 0; i < PROTOCOL_FRAME_SIZE; ++i) {
        data[i] = ready_frame[i];
    }
    frame_ready = 0;
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    *command = data[2];
    *seq = data[3];
    *param1 = (s16)((u16)data[4] | ((u16)data[5] << 8));
    *param2 = (s16)((u16)data[6] | ((u16)data[7] << 8));
    return 1;
}

void Protocol_SendStatus(u8 command, u8 seq, u8 status)
{
    u8 frame[PROTOCOL_FRAME_SIZE];
    u8 i;

    frame[0] = PROTOCOL_HEAD_0;
    frame[1] = PROTOCOL_HEAD_1;
    frame[2] = (u8)(command | 0x80U);
    frame[3] = seq;
    frame[4] = status;
    frame[5] = 0;
    frame[6] = 0;
    frame[7] = 0;
    frame[8] = Protocol_Checksum(frame);
    frame[9] = PROTOCOL_TAIL;

    for (i = 0; i < PROTOCOL_FRAME_SIZE; ++i) {
        USART_SendData(USART1, frame[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) { }
    }
}
