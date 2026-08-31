#ifndef __CONTROL_SYSTEM_H
#define __CONTROL_SYSTEM_H

#include "stm32f10x.h"

/* TIM4 PWM：72MHz / (9 + 1) / (7199 + 1) = 1kHz。 */
#define CAR_PWM_PERIOD              ((u16)7199)
#define CAR_PWM_PRESCALER           ((u16)9)

/* 闭环每 50ms 更新一次；140 脉冲/周期约对应 1 转/秒。 */
#define CAR_CONTROL_PERIOD_MS       ((u16)50)
#define CAR_CRUISE_COUNTS           ((s16)140)
#define CAR_MIN_TARGET_COUNTS       ((s16)40)

/* 每轮约 2800 个编码器计数；每 10 个控制周期输出一次速度。 */
#define CAR_ENCODER_COUNTS_PER_REV  ((s32)2800)
#define CAR_SPEED_REPORT_STEPS      ((u16)10)

/* 原动作时间为 3200ms，直行和后退均延长到 2.5 倍。 */
#define CAR_STRAIGHT_TIME_MS        ((u16)8000)
#define CAR_REVERSE_TIME_MS         ((u16)8000)
#define CAR_STOP_PAUSE_MS           ((u16)800)

/* 实车标定结果：小车前进时左右编码器原始计数均为正。 */
#define LEFT_ENCODER_FORWARD_SIGN   ((s16)1)
#define RIGHT_ENCODER_FORWARD_SIGN  ((s16)1)

/* 增量 PI 与左右轮同步纠偏参数。 */
#define CAR_PI_KP                   (7.0f)
#define CAR_PI_KI                   (0.8f)
#define CAR_PWM_FEEDFORWARD         ((s16)4200)
#define CAR_SYNC_SPEED_KP           (0.20f)
#define CAR_SYNC_POSITION_KP        (0.010f)
#define CAR_SYNC_LIMIT_COUNTS       ((s16)35)

typedef void (*ControlLedStep)(void);

void Control_System_Init(void);
void Control_SetWheels(s16 left_speed, s16 right_speed);
void Control_Stop(void);
void Control_RunTimedPID(s8 left_direction, s8 right_direction,
                         u16 run_time_ms, ControlLedStep led_effect);

#endif
