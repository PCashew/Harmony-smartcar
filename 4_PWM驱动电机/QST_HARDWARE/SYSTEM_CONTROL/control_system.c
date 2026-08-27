#include "control_system.h"

#define AIN GPIO_Pin_13
#define BIN GPIO_Pin_14

#define PWMA TIM4->CCR1
#define PWMB TIM4->CCR2

u32 myabs(long int a)
{
    u32 temp;

    if(a < 0)
        temp = -a;
    else
        temp = a;

    return temp;
}

void PWM_Init(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /* 电机方向控制引脚 PB13、PB14 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = AIN | BIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, AIN | BIN);

    /* TIM4 时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* TIM4 PWM 输出引脚 PB6、PB7 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 定时器基本参数 */
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* PWM参数 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM4, ENABLE);

    TIM_Cmd(TIM4, ENABLE);
}

void Set_Pwm(int motor_a, int motor_b)
{
    /* 电机A */
    if(motor_a > 0)
    {
        GPIO_ResetBits(GPIOB, AIN);
        PWMA = myabs(motor_a);
    }
    else if(motor_a == 0)
    {
        GPIO_SetBits(GPIOB, AIN);
        PWMA = 7199;
    }
    else
    {
        GPIO_SetBits(GPIOB, AIN);
        PWMA = 7199 - myabs(motor_a);
    }

    /* 电机B */
    if(motor_b > 0)
    {
        GPIO_ResetBits(GPIOB, BIN);
        PWMB = myabs(motor_b);
    }
    else if(motor_b == 0)
    {
        GPIO_SetBits(GPIOB, BIN);
        PWMB = 7199;
    }
    else
    {
        GPIO_SetBits(GPIOB, BIN);
        PWMB = 7199 - myabs(motor_b);
    }
}
