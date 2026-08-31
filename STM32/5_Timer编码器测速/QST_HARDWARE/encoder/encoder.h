#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* 编码器定时器采用16位满量程计数，支持正反向累计 */
#define ENCODER_TIM_PERIOD ((u16)65535)

/* 编码器接口：左轮TIM2，右轮TIM3 */
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
int Read_Encoder(u8 TIMX);

#endif
