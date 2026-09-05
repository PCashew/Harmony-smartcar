#include "control_system.h"
#include "encoder.h"
#include "motor.h"
#include "usart.h"

typedef enum {
    MODE_IDLE = 0,
    MODE_SPEED,
    MODE_POSITION,
    MODE_RETRACE
} ControlMode;

typedef struct {
    float integral;
    float previous_error;
} SpeedController;

typedef struct {
    s16 left;
    s16 right;
} PathSegment;

#define PATH_SEGMENT_CAPACITY 50U
#define PATH_SAMPLE_TICKS     10U

static volatile u16 tick_divider;
static volatile ControlMode mode;
static volatile s32 position_left;
static volatile s32 position_right;
static volatile s32 speed_sync_left;
static volatile s32 speed_sync_right;
static volatile s32 target_left;
static volatile s32 target_right;
static volatile s16 requested_left_mm_s;
static volatile s16 requested_right_mm_s;
static volatile u16 requested_max_mm_s;
static volatile u16 command_ticks;
static volatile u8 active_command;
static volatile u8 active_seq;
static volatile u8 settle_ticks;
static volatile u8 event_pending;
static volatile u8 event_command;
static volatile u8 event_seq;
static volatile u8 event_status;
static SpeedController controller_left;
static SpeedController controller_right;
static volatile PathSegment path_segments[PATH_SEGMENT_CAPACITY];
static volatile u8 path_start;
static volatile u8 path_count;
static volatile u8 path_sample_ticks;
static volatile s32 path_accum_left;
static volatile s32 path_accum_right;
static volatile s32 retrace_remaining_counts;
static volatile s32 encoder_left_counts;
static volatile s32 encoder_right_counts;

static void ResetControllers(void);

static long AbsLong(long value)
{
    return value < 0 ? -value : value;
}

static float AbsFloat(float value)
{
    return value < 0.0f ? -value : value;
}

static float ClampFloat(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static s16 ClampToS16(s32 value)
{
    if (value > 32767L) return 32767;
    if (value < -32768L) return -32768;
    return (s16)value;
}

static void PathPush(s16 left, s16 right)
{
    u8 index;
    if (AbsLong(left) + AbsLong(right) < 4L) return;
    if (path_count >= PATH_SEGMENT_CAPACITY) {
        path_start = (u8)((path_start + 1U) % PATH_SEGMENT_CAPACITY);
        path_count = PATH_SEGMENT_CAPACITY - 1U;
    }
    index = (u8)((path_start + path_count) % PATH_SEGMENT_CAPACITY);
    path_segments[index].left = left;
    path_segments[index].right = right;
    ++path_count;
}

static void PathFlushAccumulator(void)
{
    if (path_sample_ticks > 0U) {
        PathPush(ClampToS16(path_accum_left), ClampToS16(path_accum_right));
    }
    path_accum_left = 0;
    path_accum_right = 0;
    path_sample_ticks = 0;
}

static void PathRecord(s16 left, s16 right, ControlMode sample_mode)
{
    if (sample_mode == MODE_RETRACE) return;
    if (sample_mode == MODE_IDLE) {
        PathFlushAccumulator();
        return;
    }
    path_accum_left += left;
    path_accum_right += right;
    if (++path_sample_ticks >= PATH_SAMPLE_TICKS) {
        PathFlushAccumulator();
    }
}

static u8 PathPop(PathSegment *segment)
{
    u8 index;
    if (path_count == 0U) return 0;
    index = (u8)((path_start + path_count - 1U) % PATH_SEGMENT_CAPACITY);
    segment->left = path_segments[index].left;
    segment->right = path_segments[index].right;
    --path_count;
    return 1;
}

static u8 LoadNextRetraceSegment(void)
{
    PathSegment segment;
    s32 cost;
    float scale = 1.0f;

    while (PathPop(&segment)) {
        cost = (AbsLong(segment.left) + AbsLong(segment.right)) / 2L;
        if (cost < 2L) continue;
        if (cost > retrace_remaining_counts) {
            scale = (float)retrace_remaining_counts / (float)cost;
            segment.left = (s16)((float)segment.left * scale);
            segment.right = (s16)((float)segment.right * scale);
            cost = retrace_remaining_counts;
        }
        target_left = position_left - segment.left;
        target_right = position_right - segment.right;
        retrace_remaining_counts -= cost;
        settle_ticks = 0;
        ResetControllers();
        return 1;
    }
    return 0;
}

static s32 DistanceMmToCounts(s16 distance_mm)
{
    float circumference_mm;
    circumference_mm = ((float)CAR_WHEEL_DIAMETER_X100_MM / 100.0f) * 3.1415926f;
    return (s32)(((float)distance_mm * (float)CAR_ENCODER_COUNTS_PER_REV) / circumference_mm);
}

static s32 AngleToCounts(s16 angle_x10_deg)
{
    float value;
    value = ((float)angle_x10_deg * (float)CAR_EFFECTIVE_TRACK_X100_MM *
             (float)CAR_ENCODER_COUNTS_PER_REV) /
            (3600.0f * (float)CAR_WHEEL_DIAMETER_X100_MM);
    return (s32)value;
}

static float MmPerSecondToTickCounts(s16 speed_mm_s)
{
    float circumference_mm;
    circumference_mm = ((float)CAR_WHEEL_DIAMETER_X100_MM / 100.0f) * 3.1415926f;
    return ((float)speed_mm_s * (float)CAR_ENCODER_COUNTS_PER_REV *
            ((float)CAR_CONTROL_PERIOD_MS / 1000.0f)) / circumference_mm;
}

static void ResetControllers(void)
{
    controller_left.integral = 0.0f;
    controller_left.previous_error = 0.0f;
    controller_right.integral = 0.0f;
    controller_right.previous_error = 0.0f;
}

static int SpeedPI(SpeedController *controller, float target, s16 measured)
{
    float error;
    float pwm;
    float feedforward;

    if (AbsFloat(target) < 0.20f) {
        controller->integral = 0.0f;
        controller->previous_error = 0.0f;
        return 0;
    }

    error = target - (float)measured;
    controller->integral = ClampFloat(controller->integral + error, -180.0f, 180.0f);
    feedforward = target > 0.0f ? 2700.0f : -2700.0f;
    pwm = feedforward + 115.0f * error + 7.0f * controller->integral +
          18.0f * (error - controller->previous_error);
    controller->previous_error = error;
    pwm = ClampFloat(pwm, -(float)CAR_PWM_LIMIT, (float)CAR_PWM_LIMIT);
    return (int)pwm;
}

static void PublishEvent(u8 status)
{
    event_command = active_command;
    event_seq = active_seq;
    event_status = status;
    event_pending = 1;
}

static void StopInternal(void)
{
    mode = MODE_IDLE;
    requested_left_mm_s = 0;
    requested_right_mm_s = 0;
    Set_Pwm(0, 0);
    ResetControllers();
}

void Motion_Init(void)
{
    tick_divider = 0;
    position_left = 0;
    position_right = 0;
    speed_sync_left = 0;
    speed_sync_right = 0;
    event_pending = 0;
    path_start = 0;
    path_count = 0;
    path_sample_ticks = 0;
    path_accum_left = 0;
    path_accum_right = 0;
    StopInternal();
    CarLight_Stop();
}

MotionStatus Motion_Stop(u8 seq)
{
    (void)seq;
    StopInternal();
    CarLight_Stop();
    return MOTION_STATUS_DONE;
}

MotionStatus Motion_SetSpeed(s16 left_mm_s, s16 right_mm_s, u8 seq)
{
    ControlMode previous_mode;
    s16 previous_left;
    s16 previous_right;

    if (left_mm_s < -250 || left_mm_s > 250 || right_mm_s < -250 || right_mm_s > 250) {
        return MOTION_STATUS_BAD_PARAM;
    }

    __disable_irq();
    previous_mode = mode;
    previous_left = requested_left_mm_s;
    previous_right = requested_right_mm_s;
    if (previous_mode != MODE_SPEED ||
        (previous_left < 0 && left_mm_s >= 0) ||
        (previous_left > 0 && left_mm_s <= 0) ||
        (previous_right < 0 && right_mm_s >= 0) ||
        (previous_right > 0 && right_mm_s <= 0)) {
        speed_sync_left = 0;
        speed_sync_right = 0;
    }

    requested_left_mm_s = left_mm_s;
    requested_right_mm_s = right_mm_s;
    active_command = CMD_SET_SPEED;
    active_seq = seq;
    command_ticks = 0;
    mode = (left_mm_s == 0 && right_mm_s == 0) ? MODE_IDLE : MODE_SPEED;
    __enable_irq();
    if (mode == MODE_IDLE) {
        Set_Pwm(0, 0);
        CarLight_Stop();
    } else if (left_mm_s >= 0 && right_mm_s >= 0) {
        CarLight_Forward();
    } else if (left_mm_s <= 0 && right_mm_s <= 0) {
        CarLight_Backward();
    } else if (left_mm_s < right_mm_s) {
        CarLight_TurnLeft();
    } else {
        CarLight_TurnRight();
    }
    return MOTION_STATUS_ACCEPTED;
}

static MotionStatus StartPosition(s32 left_counts, s32 right_counts, u16 max_speed_mm_s,
                                  u8 command, u8 seq)
{
    if (mode == MODE_POSITION || mode == MODE_RETRACE) return MOTION_STATUS_BUSY;
    if (max_speed_mm_s < 50 || max_speed_mm_s > 220 ||
        (left_counts == 0 && right_counts == 0)) {
        return MOTION_STATUS_BAD_PARAM;
    }

    position_left = 0;
    position_right = 0;
    target_left = left_counts;
    target_right = right_counts;
    requested_max_mm_s = max_speed_mm_s;
    active_command = command;
    active_seq = seq;
    command_ticks = 0;
    settle_ticks = 0;
    ResetControllers();
    mode = MODE_POSITION;
    return MOTION_STATUS_ACCEPTED;
}

MotionStatus Motion_MoveDistance(s16 distance_mm, u16 max_speed_mm_s, u8 seq)
{
    s32 counts;
    if (distance_mm < -1000 || distance_mm > 1000 || distance_mm == 0) {
        return MOTION_STATUS_BAD_PARAM;
    }
    counts = DistanceMmToCounts(distance_mm);
    if (StartPosition(counts, counts, max_speed_mm_s, CMD_MOVE_DISTANCE, seq) !=
        MOTION_STATUS_ACCEPTED) {
        return mode == MODE_POSITION ? MOTION_STATUS_BUSY : MOTION_STATUS_BAD_PARAM;
    }
    if (distance_mm > 0) CarLight_Forward();
    else CarLight_Backward();
    return MOTION_STATUS_ACCEPTED;
}

MotionStatus Motion_TurnAngle(s16 angle_x10_deg, u16 max_speed_mm_s, u8 seq)
{
    s32 counts;
    if (angle_x10_deg < -1800 || angle_x10_deg > 1800 || angle_x10_deg == 0) {
        return MOTION_STATUS_BAD_PARAM;
    }
    counts = AngleToCounts(angle_x10_deg);
    if (StartPosition(-counts, counts, max_speed_mm_s, CMD_TURN_ANGLE, seq) !=
        MOTION_STATUS_ACCEPTED) {
        return mode == MODE_POSITION ? MOTION_STATUS_BUSY : MOTION_STATUS_BAD_PARAM;
    }
    if (angle_x10_deg > 0) CarLight_TurnLeft();
    else CarLight_TurnRight();
    return MOTION_STATUS_ACCEPTED;
}

MotionStatus Motion_RetracePath(u16 max_distance_mm, u16 max_speed_mm_s, u8 seq)
{
    s32 requested_counts;
    if (max_distance_mm < 20U || max_distance_mm > 300U ||
        max_speed_mm_s < 50U || max_speed_mm_s > 150U) {
        return MOTION_STATUS_BAD_PARAM;
    }

    __disable_irq();
    if (mode == MODE_POSITION || mode == MODE_RETRACE) {
        __enable_irq();
        return MOTION_STATUS_BUSY;
    }
    PathFlushAccumulator();
    if (path_count == 0U) {
        __enable_irq();
        return MOTION_STATUS_BAD_PARAM;
    }

    requested_counts = DistanceMmToCounts((s16)max_distance_mm);
    position_left = 0;
    position_right = 0;
    requested_max_mm_s = max_speed_mm_s;
    retrace_remaining_counts = requested_counts;
    active_command = CMD_RETRACE_PATH;
    active_seq = seq;
    command_ticks = 0;
    settle_ticks = 0;
    mode = MODE_RETRACE;
    if (!LoadNextRetraceSegment()) {
        mode = MODE_IDLE;
        __enable_irq();
        return MOTION_STATUS_BAD_PARAM;
    }
    __enable_irq();
    CarLight_Backward();
    return MOTION_STATUS_ACCEPTED;
}

static float PositionTarget(s32 error, float max_counts)
{
    float target;
    target = (float)error * 0.10f;
    target = ClampFloat(target, -max_counts, max_counts);
    if (error > CAR_POSITION_TOLERANCE && target < 2.5f) target = 2.5f;
    if (error < -CAR_POSITION_TOLERANCE && target > -2.5f) target = -2.5f;
    if (AbsLong(error) <= CAR_POSITION_TOLERANCE) target = 0.0f;
    return target;
}

void Motion_ControlTick(void)
{
    s16 measured_left;
    s16 measured_right;
    s32 error_left;
    s32 error_right;
    s32 progress_left;
    s32 progress_right;
    float target_speed_left;
    float target_speed_right;
    float max_counts;
    float sync_error;
    float correction;
    int pwm_left;
    int pwm_right;
    ControlMode sample_mode;

    sample_mode = mode;
    measured_left = (s16)Read_Encoder(2);
    measured_right = (s16)Read_Encoder(3);
    encoder_left_counts += measured_left;
    encoder_right_counts += measured_right;
    PathRecord(measured_left, measured_right, sample_mode);
    position_left += measured_left;
    position_right += measured_right;
    if (sample_mode == MODE_SPEED) {
        speed_sync_left += measured_left;
        speed_sync_right += measured_right;
    }

    if (mode == MODE_IDLE) {
        Set_Pwm(0, 0);
        return;
    }

    ++command_ticks;
    if ((mode == MODE_SPEED && command_ticks > 50U) ||
        ((mode == MODE_POSITION || mode == MODE_RETRACE) && command_ticks > 1500U)) {
        StopInternal();
        PublishEvent(MOTION_STATUS_TIMEOUT);
        return;
    }

    if (mode == MODE_SPEED) {
        target_speed_left = MmPerSecondToTickCounts(requested_left_mm_s);
        target_speed_right = MmPerSecondToTickCounts(requested_right_mm_s);
        if ((requested_left_mm_s > 0 && requested_right_mm_s > 0) ||
            (requested_left_mm_s < 0 && requested_right_mm_s < 0)) {
            sync_error = (float)(speed_sync_left - speed_sync_right);
            target_speed_left -= ClampFloat(sync_error * 0.012f, -2.0f, 2.0f);
            target_speed_right += ClampFloat(sync_error * 0.012f, -2.0f, 2.0f);
        }
    } else {
        error_left = target_left - position_left;
        error_right = target_right - position_right;
        max_counts = AbsFloat(MmPerSecondToTickCounts((s16)requested_max_mm_s));
        target_speed_left = PositionTarget(error_left, max_counts);
        target_speed_right = PositionTarget(error_right, max_counts);

        progress_left = target_left >= 0 ? position_left : -position_left;
        progress_right = target_right >= 0 ? position_right : -position_right;
        sync_error = (float)(progress_left - progress_right);
        if (AbsLong(error_left) > CAR_POSITION_TOLERANCE &&
            AbsLong(error_right) > CAR_POSITION_TOLERANCE) {
            correction = ClampFloat(sync_error * 0.015f, -2.0f, 2.0f);
            target_speed_left -= target_left >= 0 ? correction : -correction;
            target_speed_right += target_right >= 0 ? correction : -correction;
        }

        if (AbsLong(error_left) <= CAR_POSITION_TOLERANCE &&
            AbsLong(error_right) <= CAR_POSITION_TOLERANCE &&
            AbsLong(measured_left) <= 2 && AbsLong(measured_right) <= 2) {
            if (++settle_ticks >= CAR_SETTLE_TICKS) {
                if (mode == MODE_RETRACE && retrace_remaining_counts > 0L &&
                    LoadNextRetraceSegment()) {
                    Set_Pwm(0, 0);
                    return;
                }
                StopInternal();
                PublishEvent(MOTION_STATUS_DONE);
                return;
            }
        } else {
            settle_ticks = 0;
        }
    }

    pwm_left = SpeedPI(&controller_left, target_speed_left, measured_left);
    pwm_right = SpeedPI(&controller_right, target_speed_right, measured_right);
    Set_Pwm(pwm_left, pwm_right);
}

u8 Motion_GetEvent(u8 *command, u8 *seq, u8 *status)
{
    if (!event_pending) return 0;
    __disable_irq();
    *command = event_command;
    *seq = event_seq;
    *status = event_status;
    event_pending = 0;
    __enable_irq();
    CarLight_Stop();
    return 1;
}

void Motion_GetEncoderMm(s16 *left_mm, s16 *right_mm)
{
    s32 left_counts;
    s32 right_counts;
    float circumference_mm;

    __disable_irq();
    left_counts = encoder_left_counts;
    right_counts = encoder_right_counts;
    __enable_irq();
    circumference_mm = ((float)CAR_WHEEL_DIAMETER_X100_MM / 100.0f) * 3.1415926f;
    *left_mm = ClampToS16((s32)((float)left_counts * circumference_mm /
                               (float)CAR_ENCODER_COUNTS_PER_REV));
    *right_mm = ClampToS16((s32)((float)right_counts * circumference_mm /
                                (float)CAR_ENCODER_COUNTS_PER_REV));
}

void SysTick_Handler(void)
{
    if (++tick_divider >= CAR_CONTROL_PERIOD_MS) {
        tick_divider = 0;
        Motion_ControlTick();
    }
}
