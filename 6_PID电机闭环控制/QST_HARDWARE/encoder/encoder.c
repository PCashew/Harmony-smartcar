#include "encoder.h"

/**************************************************************************
函数功能：配置指定定时器的两个输入通道滤波器
入口参数：TIM2 或 TIM3
返回值  ：无
**************************************************************************/
static void Encoder_ConfigInputFilter(TIM_TypeDef *timer)
{
    TIM_ICInitTypeDef input_capture;

    TIM_ICStructInit(&input_capture);
    input_capture.TIM_ICPolarity = TIM_ICPolarity_Rising;
    input_capture.TIM_ICSelection = TIM_ICSelection_DirectTI;
    input_capture.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    input_capture.TIM_ICFilter = 10;

    input_capture.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(timer, &input_capture);
    input_capture.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(timer, &input_capture);
}

/**************************************************************************
函数功能：把 TIM2 初始化为左轮正交编码器接口
入口参数：无
返回值  ：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
    TIM_TimeBaseInitTypeDef timer;
    GPIO_InitTypeDef gpio;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 0;
    timer.TIM_Period = ENCODER_TIM_PERIOD;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &timer);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    Encoder_ConfigInputFilter(TIM2);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

/**************************************************************************
函数功能：把 TIM3 初始化为右轮正交编码器接口
入口参数：无
返回值  ：无
**************************************************************************/
void Encoder_Init_TIM3(void)
{
    TIM_TimeBaseInitTypeDef timer;
    GPIO_InitTypeDef gpio;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = 0;
    timer.TIM_Period = ENCODER_TIM_PERIOD;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &timer);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    Encoder_ConfigInputFilter(TIM3);
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

void Encoder_Reset(void)
{
    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM3, 0);
}

/**************************************************************************
函数功能：读取一个控制周期内的有符号编码器计数并立即清零
入口参数：2 读取左轮 TIM2，3 读取右轮 TIM3
返回值  ：本周期正交编码器计数；参数非法时返回 0
**************************************************************************/
int Read_Encoder(u8 timer_number)
{
    s16 count;

    if (timer_number == 2)
    {
        count = (s16)TIM_GetCounter(TIM2);
        TIM_SetCounter(TIM2, 0);
        return (int)count;
    }

    if (timer_number == 3)
    {
        count = (s16)TIM_GetCounter(TIM3);
        TIM_SetCounter(TIM3, 0);
        return (int)count;
    }

    return 0;
}
