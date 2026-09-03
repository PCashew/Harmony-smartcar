#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
#define PWMA   TIM4->CCR1  //  pb6

#define AIN   PBout(13)
#define BIN   PBout(14)
#define PWMB   TIM4->CCR2  //    pb7
void PWM_Init(u16 arr,u16 psc);
void Motor_Init(void);
u32 myabs(long int a);
void Set_Pwm(int moto1,int moto2);

#endif
