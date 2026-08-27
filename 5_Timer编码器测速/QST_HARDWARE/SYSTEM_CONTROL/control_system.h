#ifndef __CONTROL_SYSTEM_H
#define __CONTROL_SYSTEM_H

#include "stm32f10x.h"
#include "encoder.h"
#include <stdio.h>

/* 每100ms读取一次编码器，速度值表示该周期内的有符号脉冲数 */
#define SPEED_SAMPLE_PERIOD_MS ((int)100)

extern int L_speed;
extern int R_speed;
extern int OverflowTime;
extern volatile uint32_t millis;

void System_Control(void);

#endif
