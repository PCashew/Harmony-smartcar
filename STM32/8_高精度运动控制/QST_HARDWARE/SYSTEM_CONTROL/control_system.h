#ifndef __CONTROL_SYSTEM_H
#define __CONTROL_SYSTEM_H

#include "stm32f10x.h"

/* Mechanical calibration: edit these two values after the calibration test. */
#define CAR_ENCODER_COUNTS_PER_REV   2800L
#define CAR_WHEEL_DIAMETER_X100_MM   6500L   /* 65.00 mm initial value */
#define CAR_EFFECTIVE_TRACK_X100_MM  13500L  /* 135.00 mm initial value */

#define CAR_CONTROL_PERIOD_MS        10U
#define CAR_PWM_LIMIT                7199
#define CAR_POSITION_TOLERANCE       10L
#define CAR_SETTLE_TICKS             8U

typedef enum {
    MOTION_STATUS_ACCEPTED = 1,
    MOTION_STATUS_DONE = 2,
    MOTION_STATUS_BAD_PARAM = 3,
    MOTION_STATUS_BUSY = 4,
    MOTION_STATUS_TIMEOUT = 5
} MotionStatus;

void Motion_Init(void);
void Motion_ControlTick(void);
MotionStatus Motion_Stop(u8 seq);
MotionStatus Motion_SetSpeed(s16 left_mm_s, s16 right_mm_s, u8 seq);
MotionStatus Motion_MoveDistance(s16 distance_mm, u16 max_speed_mm_s, u8 seq);
MotionStatus Motion_TurnAngle(s16 angle_x10_deg, u16 max_speed_mm_s, u8 seq);
MotionStatus Motion_RetracePath(u16 max_distance_mm, u16 max_speed_mm_s, u8 seq);
u8 Motion_GetEvent(u8 *command, u8 *seq, u8 *status);
void Motion_GetEncoderMm(s16 *left_mm, s16 *right_mm);

#endif
