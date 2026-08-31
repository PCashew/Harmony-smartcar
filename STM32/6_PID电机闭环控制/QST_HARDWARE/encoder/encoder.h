#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* TIM2/TIM3 均使用 16 位满量程编码器计数。 */
#define ENCODER_TIM_PERIOD ((u16)65535)

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
void Encoder_Reset(void);
int Read_Encoder(u8 timer_number);

#endif
