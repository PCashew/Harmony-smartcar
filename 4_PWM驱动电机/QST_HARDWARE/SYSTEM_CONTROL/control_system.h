#ifndef __SYSTEM_CONTROL_H
#define __SYSTEM_CONTROL_H

#include "stm32f10x.h"
#include "sys.h"

void PWM_Init(u16 arr, u16 psc);
void Set_Pwm(int motor_a, int motor_b);
u32 myabs(long int a);

#endif
