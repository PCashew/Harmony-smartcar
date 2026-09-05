#include <stdio.h>
#include <stdint.h>
#include "wifiiot_uart.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "hi_io.h"
#include "hi_time.h"
#include "hi_task.h"

#define IR_LEFT_GPIO                       13
#define IR_RIGHT_GPIO                      14

#define PROTOCOL_SIZE                      10
#define PROTOCOL_HEAD_0                  0xA5
#define PROTOCOL_HEAD_1                  0x5A
#define PROTOCOL_TAIL                    0x0D
#define CMD_STOP                         0x01
#define CMD_SET_SPEED                    0x02
#define CMD_MOVE_DISTANCE                0x03
#define CMD_TURN_ANGLE                   0x04
#define CMD_GET_ENCODERS                 0x07
#define STATUS_DONE                      0x02

#define LINE_DEBUG                          1
#define SENSOR_SAMPLE_MS                    5
#define SENSOR_FILTER_COUNT                 5
#define SENSOR_FILTER_THRESHOLD             3
#define DEBUG_PERIOD_SAMPLES               40

/* All values below need final calibration on the real course. */
#define SOFT_ADJUST_SAMPLES                 3
#define NORMAL_ADJUST_SAMPLES              10
#define LOST_MEMORY_SAMPLES                 2
#define EVENT_WINDOW_SAMPLES               20
#define EVENT_SIDE_MIN_HITS                 3
#define EVENT_REARM_WHITE_SAMPLES           5
#define JUNCTION_LOCK_SAMPLES             200
#define START_FINISH_GAP_MAX_SAMPLES      500
#define MIN_RACE_SAMPLES                  800

#define BASE_SPEED_MM_S                    80
#define START_SPEED_MM_S                   60
#define SOFT_INNER_SPEED_MM_S              70
#define SOFT_OUTER_SPEED_MM_S              90
#define NORMAL_INNER_SPEED_MM_S            58
#define NORMAL_OUTER_SPEED_MM_S           100
#define STRONG_INNER_SPEED_MM_S            38
#define STRONG_OUTER_SPEED_MM_S           115
#define SPEED_HEARTBEAT_SAMPLES            20

#define JUNCTION_CENTER_MM                 35
#define DEAD_END_PROBE_MM                  30
#define POSITION_SPEED_MM_S                70
#define SEARCH_BLACK_SAMPLES                2
#define SEARCH_WHITE_SAMPLES                2
#define SEARCH_TIMEOUT_SAMPLES            600
#define CENTER_BASE_MM_S                   45
#define CENTER_DELTA_MM_S                  10
#define CENTER_BLACK_SAMPLES                3
#define CENTER_ENCODER_POLL_SAMPLES         5
#define CENTER_MAX_METRIC_MM               40
#define CENTER_TIMEOUT_SAMPLES            300
#define TURN_BACK_X10_DEG                 1800
#define COMMAND_TIMEOUT_MS               15000

#define SEEN_LEFT                        0x01
#define SEEN_RIGHT                       0x02
#define ROUTE_COUNT                         3

typedef enum { DIR_LEFT = -1, DIR_RIGHT = 1 } Direction;

typedef enum {
    ACTION_FORWARD,
    ACTION_LEFT,
    ACTION_RIGHT,
    ACTION_SPECIAL
} FollowAction;

typedef enum {
    STATE_WAIT_START,
    STATE_FOLLOW,
    STATE_EVENT_CANDIDATE,
    STATE_EVENT_CHECK,
    STATE_JUNCTION,
    STATE_MOVE_TO_JUNCTION_CENTER,
    STATE_SEARCH_LEFT_WAIT_BLACK,
    STATE_SEARCH_LEFT_ON_BLACK,
    STATE_SEARCH_LEFT_EXIT,
    STATE_SEARCH_RIGHT_WAIT_BLACK,
    STATE_SEARCH_RIGHT_ON_BLACK,
    STATE_SEARCH_RIGHT_EXIT,
    STATE_CENTER_SCAN_OPPOSITE,
    STATE_CENTER_RETURN_HALF,
    STATE_CENTER_DONE,
    STATE_CENTER_FAIL,
    STATE_SEARCH_FAIL,
    STATE_DEAD_END,
    STATE_DEAD_END_PROBE,
    STATE_TURN_BACK,
    STATE_RETURNING,
    STATE_RETURN_JUNCTION,
    STATE_FINISH,
    STATE_FAULT
} CarState;

typedef struct {
    uint8_t leftHistory;
    uint8_t rightHistory;
    uint8_t samples;
} SensorFilter;

typedef struct {
    uint8_t active;
    uint8_t armed;
    uint8_t seenMask;
    uint8_t leftHits;
    uint8_t rightHits;
    uint16_t samples;
    uint8_t clearSamples;
} EventWindow;

typedef struct {
    uint8_t active;
    uint8_t command;
    uint8_t seq;
    uint8_t seenVersion;
    uint16_t samples;
} MotionWait;

static const Direction g_route[ROUTE_COUNT] = {
    DIR_LEFT, DIR_RIGHT, DIR_RIGHT
};
static volatile uint8_t g_ackCommand;
static volatile uint8_t g_ackSeq;
static volatile uint8_t g_ackStatus;
static volatile uint8_t g_ackVersion;
static volatile int16_t g_encoderLeftMm;
static volatile int16_t g_encoderRightMm;
static volatile uint8_t g_encoderVersion;
static uint8_t g_nextSeq = 1;
static int16_t g_lastLeftSpeed = 32767;
static int16_t g_lastRightSpeed = 32767;
static uint16_t g_speedHeartbeat;
static SensorFilter g_filter;
static EventWindow g_event;
static MotionWait g_motion;

static uint8_t Checksum(const uint8_t *frame)
{
    uint8_t value = 0;
    unsigned int i;
    for (i = 0; i < 8; ++i) value ^= frame[i];
    return value;
}

static uint8_t SendCommand(uint8_t command, int16_t param1, int16_t param2)
{
    uint8_t frame[PROTOCOL_SIZE];
    uint8_t seq = g_nextSeq++;
    if (g_nextSeq == 0) g_nextSeq = 1;
    frame[0] = PROTOCOL_HEAD_0;
    frame[1] = PROTOCOL_HEAD_1;
    frame[2] = command;
    frame[3] = seq;
    frame[4] = (uint8_t)((uint16_t)param1 & 0xffU);
    frame[5] = (uint8_t)(((uint16_t)param1 >> 8) & 0xffU);
    frame[6] = (uint8_t)((uint16_t)param2 & 0xffU);
    frame[7] = (uint8_t)(((uint16_t)param2 >> 8) & 0xffU);
    frame[8] = Checksum(frame);
    frame[9] = PROTOCOL_TAIL;
    (void)UartWrite(WIFI_IOT_UART_IDX_2, frame, sizeof(frame));
    return seq;
}

static void StopCar(void)
{
    (void)SendCommand(CMD_STOP, 0, 0);
    hi_sleep(5);
    (void)SendCommand(CMD_STOP, 0, 0);
    g_lastLeftSpeed = 32767;
    g_lastRightSpeed = 32767;
}

static void SetSpeed(int16_t left, int16_t right)
{
    if (left != g_lastLeftSpeed || right != g_lastRightSpeed ||
        ++g_speedHeartbeat >= SPEED_HEARTBEAT_SAMPLES) {
        (void)SendCommand(CMD_SET_SPEED, left, right);
        g_lastLeftSpeed = left;
        g_lastRightSpeed = right;
        g_speedHeartbeat = 0;
    }
}

static void StartMotion(uint8_t command, int16_t value, int16_t speed)
{
    g_lastLeftSpeed = 32767;
    g_lastRightSpeed = 32767;
    g_motion.active = 1;
    g_motion.command = command;
    g_motion.seenVersion = g_ackVersion;
    g_motion.samples = 0;
    g_motion.seq = SendCommand(command, value, speed);
}

/* 1=done, 0=running, -1=error/timeout. Never blocks sensor updates. */
static int MotionUpdate(void)
{
    if (!g_motion.active) return -1;
    if (g_ackVersion != g_motion.seenVersion) {
        g_motion.seenVersion = g_ackVersion;
        if (g_ackCommand == (uint8_t)(g_motion.command | 0x80U) &&
            g_ackSeq == g_motion.seq) {
            if (g_ackStatus == STATUS_DONE) {
                g_motion.active = 0;
                return 1;
            }
            if (g_ackStatus > STATUS_DONE) {
                g_motion.active = 0;
                return -1;
            }
        }
    }
    if (++g_motion.samples >= (COMMAND_TIMEOUT_MS / SENSOR_SAMPLE_MS)) {
        g_motion.active = 0;
        return -1;
    }
    return 0;
}

static uint16_t EncoderTurnMetric(int16_t startLeft, int16_t startRight,
                                  int16_t nowLeft, int16_t nowRight)
{
    int32_t delta = ((int32_t)nowLeft - startLeft) -
                    ((int32_t)nowRight - startRight);
    return (uint16_t)(delta < 0 ? -delta : delta);
}

static void RequestEncoders(void)
{
    (void)SendCommand(CMD_GET_ENCODERS, 0, 0);
}

static void UartReceiveTask(void *argument)
{
    uint8_t frame[PROTOCOL_SIZE];
    unsigned int index = 0;
    uint8_t byte;
    int result;
    (void)argument;
    while (1) {
        result = UartRead(WIFI_IOT_UART_IDX_2, &byte, 1);
        if (result <= 0) {
            hi_sleep(1);
            continue;
        }
        if (index == 0 && byte != PROTOCOL_HEAD_0) continue;
        if (index == 1 && byte != PROTOCOL_HEAD_1) {
            index = (byte == PROTOCOL_HEAD_0) ? 1U : 0U;
            frame[0] = byte;
            continue;
        }
        frame[index++] = byte;
        if (index < PROTOCOL_SIZE) continue;
        index = 0;
        if (frame[9] != PROTOCOL_TAIL || frame[8] != Checksum(frame)) continue;
        if (frame[2] == (uint8_t)(CMD_GET_ENCODERS | 0x80U)) {
            g_encoderLeftMm = (int16_t)((uint16_t)frame[4] |
                              ((uint16_t)frame[5] << 8));
            g_encoderRightMm = (int16_t)((uint16_t)frame[6] |
                               ((uint16_t)frame[7] << 8));
            ++g_encoderVersion;
            continue;
        }
        g_ackCommand = frame[2];
        g_ackSeq = frame[3];
        g_ackStatus = frame[4];
        ++g_ackVersion;
    }
}

static uint8_t BitCount5(uint8_t value)
{
    uint8_t count = 0;
    value &= 0x1fU;
    while (value != 0) {
        count += value & 1U;
        value >>= 1;
    }
    return count;
}

static FollowAction Classify(uint8_t leftBlack, uint8_t rightBlack)
{
    if (leftBlack && rightBlack) return ACTION_SPECIAL;
    if (leftBlack) return ACTION_LEFT;
    if (rightBlack) return ACTION_RIGHT;
    return ACTION_FORWARD;
}

static int ReadFiltered(FollowAction *action)
{
    WifiIotGpioValue left;
    WifiIotGpioValue right;
    uint8_t mask = (1U << SENSOR_FILTER_COUNT) - 1U;
    if (GpioGetInputVal(IR_LEFT_GPIO, &left) != 0 ||
        GpioGetInputVal(IR_RIGHT_GPIO, &right) != 0) return -1;
    g_filter.leftHistory = (uint8_t)(((g_filter.leftHistory << 1) |
        (left == WIFI_IOT_GPIO_VALUE0 ? 1U : 0U)) & mask);
    g_filter.rightHistory = (uint8_t)(((g_filter.rightHistory << 1) |
        (right == WIFI_IOT_GPIO_VALUE0 ? 1U : 0U)) & mask);
    if (g_filter.samples < SENSOR_FILTER_COUNT) ++g_filter.samples;
    *action = Classify(BitCount5(g_filter.leftHistory) >= SENSOR_FILTER_THRESHOLD,
                       BitCount5(g_filter.rightHistory) >= SENSOR_FILTER_THRESHOLD);
    return 0;
}

static void EventReset(uint8_t armed)
{
    g_event.active = 0;
    g_event.armed = armed;
    g_event.seenMask = 0;
    g_event.leftHits = 0;
    g_event.rightHits = 0;
    g_event.samples = 0;
    g_event.clearSamples = 0;
}

static int EventUpdate(FollowAction action)
{
    uint8_t leftBlack = action == ACTION_LEFT || action == ACTION_SPECIAL;
    uint8_t rightBlack = action == ACTION_RIGHT || action == ACTION_SPECIAL;
    if (!g_event.armed) {
        if (action == ACTION_FORWARD) {
            if (++g_event.clearSamples >= EVENT_REARM_WHITE_SAMPLES) EventReset(1);
        } else {
            g_event.clearSamples = 0;
        }
        return 0;
    }
    if (!g_event.active) {
        if (!leftBlack && !rightBlack) return 0;
        g_event.active = 1;
    }
    if (leftBlack) {
        g_event.seenMask |= SEEN_LEFT;
        if (g_event.leftHits < 255U) ++g_event.leftHits;
    }
    if (rightBlack) {
        g_event.seenMask |= SEEN_RIGHT;
        if (g_event.rightHits < 255U) ++g_event.rightHits;
    }
    if (++g_event.samples < EVENT_WINDOW_SAMPLES) return 0;
    if (g_event.seenMask == (SEEN_LEFT | SEEN_RIGHT) &&
        g_event.leftHits >= EVENT_SIDE_MIN_HITS &&
        g_event.rightHits >= EVENT_SIDE_MIN_HITS) {
        EventReset(0);
        return 1;
    }
    EventReset(1);
    return 0;
}

static const char *StateName(CarState state)
{
    switch (state) {
    case STATE_WAIT_START: return "WAIT_START";
    case STATE_FOLLOW: return "FOLLOW";
    case STATE_EVENT_CANDIDATE: return "EVENT_CANDIDATE";
    case STATE_EVENT_CHECK: return "EVENT_CHECK";
    case STATE_JUNCTION: return "JUNCTION";
    case STATE_MOVE_TO_JUNCTION_CENTER: return "MOVE_TO_JUNCTION_CENTER";
    case STATE_SEARCH_LEFT_WAIT_BLACK: return "SEARCH_LEFT_WAIT_BLACK";
    case STATE_SEARCH_LEFT_ON_BLACK: return "SEARCH_LEFT_ON_BLACK";
    case STATE_SEARCH_LEFT_EXIT: return "SEARCH_LEFT_EXIT";
    case STATE_SEARCH_RIGHT_WAIT_BLACK: return "SEARCH_RIGHT_WAIT_BLACK";
    case STATE_SEARCH_RIGHT_ON_BLACK: return "SEARCH_RIGHT_ON_BLACK";
    case STATE_SEARCH_RIGHT_EXIT: return "SEARCH_RIGHT_EXIT";
    case STATE_CENTER_SCAN_OPPOSITE: return "CENTER_SCAN_OPPOSITE";
    case STATE_CENTER_RETURN_HALF: return "CENTER_RETURN_HALF";
    case STATE_CENTER_DONE: return "CENTER_DONE";
    case STATE_CENTER_FAIL: return "CENTER_FAIL";
    case STATE_SEARCH_FAIL: return "SEARCH_FAIL";
    case STATE_DEAD_END: return "DEAD_END";
    case STATE_DEAD_END_PROBE: return "DEAD_END_PROBE";
    case STATE_TURN_BACK: return "TURN_BACK";
    case STATE_RETURNING: return "RETURNING";
    case STATE_RETURN_JUNCTION: return "RETURN_JUNCTION";
    case STATE_FINISH: return "FINISH";
    default: return "FAULT";
    }
}

static void Transition(CarState *state, CarState next)
{
    printf("[STATE] %s -> %s\r\n", StateName(*state), StateName(next));
    *state = next;
}

static void StopWithReason(const char *reason)
{
    printf("[STOP] %s\r\n", reason);
    StopCar();
}

static const char *ActionName(FollowAction action)
{
    switch (action) {
    case ACTION_LEFT: return "BW";
    case ACTION_RIGHT: return "WB";
    case ACTION_SPECIAL: return "BB";
    default: return "WW";
    }
}

static void FollowLine(FollowAction action, FollowAction *lastDirection,
                       uint16_t *holdSamples, uint16_t *lostSamples)
{
    if (action == ACTION_LEFT || action == ACTION_RIGHT) {
        if (action == *lastDirection) {
            if (*holdSamples < 0xffffU) ++*holdSamples;
        } else {
            *lastDirection = action;
            *holdSamples = 1;
        }
        *lostSamples = 0;
    } else if (action == ACTION_FORWARD) {
        ++*lostSamples;
    }
    if (action == ACTION_LEFT) {
        if (*holdSamples <= SOFT_ADJUST_SAMPLES)
            SetSpeed(SOFT_INNER_SPEED_MM_S, SOFT_OUTER_SPEED_MM_S);
        else if (*holdSamples <= NORMAL_ADJUST_SAMPLES)
            SetSpeed(NORMAL_INNER_SPEED_MM_S, NORMAL_OUTER_SPEED_MM_S);
        else
            SetSpeed(STRONG_INNER_SPEED_MM_S, STRONG_OUTER_SPEED_MM_S);
    } else if (action == ACTION_RIGHT) {
        if (*holdSamples <= SOFT_ADJUST_SAMPLES)
            SetSpeed(SOFT_OUTER_SPEED_MM_S, SOFT_INNER_SPEED_MM_S);
        else if (*holdSamples <= NORMAL_ADJUST_SAMPLES)
            SetSpeed(NORMAL_OUTER_SPEED_MM_S, NORMAL_INNER_SPEED_MM_S);
        else
            SetSpeed(STRONG_OUTER_SPEED_MM_S, STRONG_INNER_SPEED_MM_S);
    } else if (*lostSamples <= LOST_MEMORY_SAMPLES && *lastDirection == ACTION_LEFT) {
        SetSpeed(SOFT_INNER_SPEED_MM_S, SOFT_OUTER_SPEED_MM_S);
    } else if (*lostSamples <= LOST_MEMORY_SAMPLES && *lastDirection == ACTION_RIGHT) {
        SetSpeed(SOFT_OUTER_SPEED_MM_S, SOFT_INNER_SPEED_MM_S);
    } else {
        SetSpeed(BASE_SPEED_MM_S, BASE_SPEED_MM_S);
        if (*lostSamples > LOST_MEMORY_SAMPLES) {
            *lastDirection = ACTION_FORWARD;
            *holdSamples = 0;
        }
    }
}

static int LogicSelfCheck(void)
{
    return Classify(0, 0) == ACTION_FORWARD &&
           Classify(1, 0) == ACTION_LEFT &&
           Classify(0, 1) == ACTION_RIGHT &&
           Classify(1, 1) == ACTION_SPECIAL &&
           g_route[0] == DIR_LEFT && g_route[1] == DIR_RIGHT &&
           g_route[2] == DIR_RIGHT && CENTER_BASE_MM_S > CENTER_DELTA_MM_S;
}

static void RouteTask(void *argument)
{
    CarState state = STATE_WAIT_START;
    FollowAction action = ACTION_FORWARD;
    FollowAction lastDirection = ACTION_FORWARD;
    uint16_t correctionSamples = 0;
    uint16_t lostSamples = LOST_MEMORY_SAMPLES + 1;
    uint16_t lockSamples = 0;
    uint16_t markerGapSamples = 0;
    uint16_t raceSamples = 0;
    uint16_t debugSamples = 0;
    uint16_t searchSamples = 0;
    uint8_t searchBlackSamples = 0;
    uint8_t searchWhiteSamples = 0;
    uint8_t markerCount = 0;
    uint8_t routeIndex = 0;
    uint8_t returningFromDeadEnd = 0;
    uint8_t branchValid = 0;
    FollowAction lastLoggedAction = (FollowAction)255;
    Direction searchTarget = DIR_LEFT;
    Direction branchTaken = DIR_LEFT;
    Direction centerDirection = DIR_LEFT;
    uint16_t centerSamples = 0;
    uint16_t centerMetric = 0;
    int16_t centerStartLeft = 0;
    int16_t centerStartRight = 0;
    int16_t centerReturnLeft = 0;
    int16_t centerReturnRight = 0;
    uint8_t centerStartValid = 0;
    uint8_t centerBlackSamples = 0;
    uint8_t centerEncoderVersion = 0;
    uint8_t centerEncoderPoll = 0;
    (void)argument;

    if (!LogicSelfCheck()) {
        StopWithReason("SELF_CHECK_FAILED");
        printf("route logic self-check failed\r\n");
        return;
    }
    EventReset(1);
    printf("firmware: v14.2 route events and bounded active center\r\n");
    printf("route: Hi3861 sensors, STM32 encoder motion\r\n");

    while (state != STATE_FINISH && state != STATE_FAULT) {
        if (ReadFiltered(&action) != 0) {
            StopWithReason("IR_READ_ERROR");
            Transition(&state, STATE_FAULT);
            break;
        }
        if (markerCount > 0 && ++markerGapSamples > START_FINISH_GAP_MAX_SAMPLES) {
            markerCount = 0;
            markerGapSamples = 0;
            if (state != STATE_WAIT_START && routeIndex >= ROUTE_COUNT) {
                printf("[ERROR] ROUTE INDEX OVERFLOW index=%u\r\n", routeIndex);
                StopWithReason("ROUTE_INDEX_OVERFLOW");
                Transition(&state, STATE_FAULT);
            }
        }
        if (lockSamples > 0) --lockSamples;
        if (state != STATE_WAIT_START && raceSamples < 0xffffU) ++raceSamples;
#if LINE_DEBUG
        if (action != lastLoggedAction || ++debugSamples >= DEBUG_PERIOD_SAMPLES) {
            debugSamples = 0;
            lastLoggedAction = action;
            printf("[IR] %s state=%s seen=0x%02x junction=%u\r\n",
                   ActionName(action), StateName(state), g_event.seenMask,
                   routeIndex);
        }
#endif
        if (state == STATE_WAIT_START) {
            SetSpeed(START_SPEED_MM_S, START_SPEED_MM_S);
            if (EventUpdate(action)) {
                if (++markerCount >= 2) {
                    markerCount = 0;
                    markerGapSamples = 0;
                    raceSamples = 0;
                    lockSamples = JUNCTION_LOCK_SAMPLES;
                    printf("[EVENT] Double line detected: race started\r\n");
                    Transition(&state, STATE_FOLLOW);
                } else {
                    markerGapSamples = 0;
                    printf("[EVENT] Start marker first bar\r\n");
                }
            }
        } else if (state == STATE_FOLLOW || state == STATE_RETURNING) {
            FollowLine(action, &lastDirection, &correctionSamples, &lostSamples);
            if (lockSamples == 0 && EventUpdate(action)) {
                Transition(&state, state == STATE_RETURNING
                    ? STATE_RETURN_JUNCTION : STATE_EVENT_CHECK);
            }
        } else if (state == STATE_EVENT_CHECK) {
            StopWithReason("EVENT_CHECK");
            if (routeIndex < ROUTE_COUNT) {
                searchTarget = g_route[routeIndex];
                printf("[ROUTE EVENT] index=%u action=%s\r\n", routeIndex,
                       searchTarget == DIR_LEFT ? "LEFT" : "RIGHT");
                Transition(&state, STATE_JUNCTION);
            } else if (raceSamples >= MIN_RACE_SAMPLES) {
                if (++markerCount >= 2) {
                    printf("[EVENT] Double line detected: finish\r\n");
                    StopWithReason("FINISH");
                    Transition(&state, STATE_FINISH);
                } else {
                    markerGapSamples = 0;
                    printf("[EVENT] Finish marker first bar\r\n");
                    Transition(&state, STATE_FOLLOW);
                }
            } else {
                lockSamples = EVENT_WINDOW_SAMPLES;
                Transition(&state, STATE_FOLLOW);
            }
        } else if (state == STATE_JUNCTION) {
                StartMotion(CMD_MOVE_DISTANCE, JUNCTION_CENTER_MM,
                            POSITION_SPEED_MM_S);
                Transition(&state, STATE_MOVE_TO_JUNCTION_CENTER);
        } else if (state == STATE_MOVE_TO_JUNCTION_CENTER) {
            int motion = MotionUpdate();
            if (motion < 0) {
                StopWithReason("JUNCTION_CENTER_FAILED");
                Transition(&state, STATE_FAULT);
            } else if (motion > 0) {
                searchSamples = 0;
                searchBlackSamples = 0;
                searchWhiteSamples = 0;
                if (searchTarget == DIR_LEFT) {
                    Transition(&state, STATE_SEARCH_LEFT_WAIT_BLACK);
                } else {
                    Transition(&state, STATE_SEARCH_RIGHT_WAIT_BLACK);
                }
            }
        } else if (state == STATE_SEARCH_LEFT_WAIT_BLACK ||
                   state == STATE_SEARCH_RIGHT_WAIT_BLACK) {
            uint8_t targetBlack = searchTarget == DIR_LEFT
                ? (action == ACTION_LEFT || action == ACTION_SPECIAL)
                : (action == ACTION_RIGHT || action == ACTION_SPECIAL);
            SetSpeed(searchTarget == DIR_LEFT ? -POSITION_SPEED_MM_S : POSITION_SPEED_MM_S,
                     searchTarget == DIR_LEFT ? POSITION_SPEED_MM_S : -POSITION_SPEED_MM_S);
            ++searchSamples;
            if (targetBlack) {
                if (++searchBlackSamples >= SEARCH_BLACK_SAMPLES) {
                    Transition(&state, searchTarget == DIR_LEFT
                        ? STATE_SEARCH_LEFT_ON_BLACK : STATE_SEARCH_RIGHT_ON_BLACK);
                }
            } else {
                searchBlackSamples = 0;
            }
            if (searchSamples >= SEARCH_TIMEOUT_SAMPLES) {
                Transition(&state, STATE_SEARCH_FAIL);
            }
        } else if (state == STATE_SEARCH_LEFT_ON_BLACK ||
                   state == STATE_SEARCH_RIGHT_ON_BLACK) {
            SetSpeed(searchTarget == DIR_LEFT ? -POSITION_SPEED_MM_S : POSITION_SPEED_MM_S,
                     searchTarget == DIR_LEFT ? POSITION_SPEED_MM_S : -POSITION_SPEED_MM_S);
            ++searchSamples;
            if (action == ACTION_FORWARD) {
                searchWhiteSamples = 1;
                Transition(&state, searchTarget == DIR_LEFT
                    ? STATE_SEARCH_LEFT_EXIT : STATE_SEARCH_RIGHT_EXIT);
            }
            if (searchSamples >= SEARCH_TIMEOUT_SAMPLES) {
                Transition(&state, STATE_SEARCH_FAIL);
            }
        } else if (state == STATE_SEARCH_LEFT_EXIT ||
                   state == STATE_SEARCH_RIGHT_EXIT) {
            SetSpeed(searchTarget == DIR_LEFT ? -POSITION_SPEED_MM_S : POSITION_SPEED_MM_S,
                     searchTarget == DIR_LEFT ? POSITION_SPEED_MM_S : -POSITION_SPEED_MM_S);
            ++searchSamples;
            if (action == ACTION_FORWARD) {
                if (++searchWhiteSamples >= SEARCH_WHITE_SAMPLES) {
                    StopWithReason("BRANCH_ACQUIRED");
                    printf("[JUNCTION] branch acquired\r\n");
                    branchTaken = searchTarget;
                    branchValid = 1;
                    if (returningFromDeadEnd) {
                        returningFromDeadEnd = 0;
                        lockSamples = JUNCTION_LOCK_SAMPLES;
                        EventReset(0);
                        Transition(&state, STATE_FOLLOW);
                    } else {
                        centerDirection = searchTarget;
                        centerSamples = 0;
                        centerMetric = 0;
                        centerStartValid = 0;
                        centerBlackSamples = 0;
                        centerEncoderVersion = g_encoderVersion;
                        centerEncoderPoll = CENTER_ENCODER_POLL_SAMPLES;
                        EventReset(0);
                        printf("[CENTER] start after %s\r\n",
                               centerDirection == DIR_LEFT ? "LEFT" : "RIGHT");
                        Transition(&state, STATE_CENTER_SCAN_OPPOSITE);
                    }
                }
            } else {
                searchWhiteSamples = 0;
                Transition(&state, searchTarget == DIR_LEFT
                    ? STATE_SEARCH_LEFT_ON_BLACK : STATE_SEARCH_RIGHT_ON_BLACK);
            }
            if (searchSamples >= SEARCH_TIMEOUT_SAMPLES &&
                (state == STATE_SEARCH_LEFT_EXIT ||
                 state == STATE_SEARCH_RIGHT_EXIT)) {
                Transition(&state, STATE_SEARCH_FAIL);
            }
        } else if (state == STATE_CENTER_SCAN_OPPOSITE) {
            uint8_t oppositeBlack = centerDirection == DIR_RIGHT
                ? (action == ACTION_LEFT || action == ACTION_SPECIAL)
                : (action == ACTION_RIGHT || action == ACTION_SPECIAL);
            SetSpeed(centerDirection == DIR_RIGHT
                         ? CENTER_BASE_MM_S + CENTER_DELTA_MM_S
                         : CENTER_BASE_MM_S - CENTER_DELTA_MM_S,
                     centerDirection == DIR_RIGHT
                         ? CENTER_BASE_MM_S - CENTER_DELTA_MM_S
                         : CENTER_BASE_MM_S + CENTER_DELTA_MM_S);
            if (++centerEncoderPoll >= CENTER_ENCODER_POLL_SAMPLES) {
                centerEncoderPoll = 0;
                RequestEncoders();
            }
            if (g_encoderVersion != centerEncoderVersion) {
                centerEncoderVersion = g_encoderVersion;
                if (!centerStartValid) {
                    centerStartLeft = g_encoderLeftMm;
                    centerStartRight = g_encoderRightMm;
                    centerStartValid = 1;
                } else {
                    centerMetric = EncoderTurnMetric(centerStartLeft,
                        centerStartRight, g_encoderLeftMm, g_encoderRightMm);
                }
            }
            if (centerStartValid && oppositeBlack) {
                if (++centerBlackSamples >= CENTER_BLACK_SAMPLES &&
                    centerMetric > 0U) {
                    centerReturnLeft = g_encoderLeftMm;
                    centerReturnRight = g_encoderRightMm;
                    centerSamples = 0;
                    centerEncoderPoll = CENTER_ENCODER_POLL_SAMPLES;
                    printf("[CENTER] opposite edge found M=%u\r\n", centerMetric);
                    printf("[CENTER] return half\r\n");
                    Transition(&state, STATE_CENTER_RETURN_HALF);
                }
            } else {
                centerBlackSamples = 0;
            }
            if (centerMetric >= CENTER_MAX_METRIC_MM ||
                ++centerSamples >= CENTER_TIMEOUT_SAMPLES) {
                Transition(&state, STATE_CENTER_FAIL);
            }
        } else if (state == STATE_CENTER_RETURN_HALF) {
            uint16_t returnMetric;
            SetSpeed(centerDirection == DIR_RIGHT
                         ? CENTER_BASE_MM_S - CENTER_DELTA_MM_S
                         : CENTER_BASE_MM_S + CENTER_DELTA_MM_S,
                     centerDirection == DIR_RIGHT
                         ? CENTER_BASE_MM_S + CENTER_DELTA_MM_S
                         : CENTER_BASE_MM_S - CENTER_DELTA_MM_S);
            if (++centerEncoderPoll >= CENTER_ENCODER_POLL_SAMPLES) {
                centerEncoderPoll = 0;
                RequestEncoders();
            }
            returnMetric = EncoderTurnMetric(centerReturnLeft, centerReturnRight,
                                             g_encoderLeftMm, g_encoderRightMm);
            if (returnMetric >= (uint16_t)((centerMetric + 1U) / 2U)) {
                Transition(&state, STATE_CENTER_DONE);
            } else if (returnMetric >= CENTER_MAX_METRIC_MM ||
                       ++centerSamples >= CENTER_TIMEOUT_SAMPLES) {
                Transition(&state, STATE_CENTER_FAIL);
            }
        } else if (state == STATE_CENTER_DONE || state == STATE_CENTER_FAIL) {
            uint8_t oldIndex = routeIndex;
            if (state == STATE_CENTER_DONE)
                printf("[CENTER] done\r\n");
            else
                printf("[CENTER] fail\r\n");
            SetSpeed(BASE_SPEED_MM_S, BASE_SPEED_MM_S);
            centerSamples = 0;
            centerMetric = 0;
            centerStartValid = 0;
            centerBlackSamples = 0;
            searchSamples = 0;
            searchBlackSamples = 0;
            searchWhiteSamples = 0;
            correctionSamples = 0;
            lostSamples = LOST_MEMORY_SAMPLES + 1U;
            lastDirection = ACTION_FORWARD;
            ++routeIndex;
            printf("[JUNCTION] index %u -> %u\r\n", oldIndex, routeIndex);
            lockSamples = JUNCTION_LOCK_SAMPLES;
            EventReset(0);
            Transition(&state, STATE_FOLLOW);
        } else if (state == STATE_SEARCH_FAIL) {
            StopWithReason("SEARCH_FAIL");
            Transition(&state, STATE_FAULT);
        } else if (state == STATE_DEAD_END) {
            StopWithReason("DEAD_END_CONFIRMED");
            StartMotion(CMD_MOVE_DISTANCE, DEAD_END_PROBE_MM, POSITION_SPEED_MM_S);
            Transition(&state, STATE_DEAD_END_PROBE);
        } else if (state == STATE_DEAD_END_PROBE) {
            int motion = MotionUpdate();
            if (motion < 0) {
                StopWithReason("DEAD_END_PROBE_FAILED");
                Transition(&state, STATE_FAULT);
            } else if (motion > 0) {
                printf("[TURNBACK] reason=DEAD_END_CONFIRMED\r\n");
                StartMotion(CMD_TURN_ANGLE, TURN_BACK_X10_DEG, POSITION_SPEED_MM_S);
                Transition(&state, STATE_TURN_BACK);
            }
        } else if (state == STATE_TURN_BACK) {
            int motion = MotionUpdate();
            if (!branchValid || motion < 0) {
                StopWithReason("RETURN_SEARCH_FAIL");
                Transition(&state, STATE_FAULT);
            } else if (motion > 0) {
                returningFromDeadEnd = 1;
                lockSamples = JUNCTION_LOCK_SAMPLES;
                EventReset(0);
                Transition(&state, STATE_RETURNING);
            }
        } else if (state == STATE_RETURN_JUNCTION) {
            searchTarget = branchTaken == DIR_LEFT ? DIR_RIGHT : DIR_LEFT;
            printf("[JUNCTION] returning target=%s\r\n",
                   searchTarget == DIR_LEFT ? "LEFT" : "RIGHT");
            StartMotion(CMD_MOVE_DISTANCE, JUNCTION_CENTER_MM, POSITION_SPEED_MM_S);
            Transition(&state, STATE_MOVE_TO_JUNCTION_CENTER);
        }
        hi_sleep(SENSOR_SAMPLE_MS);
    }
    printf(state == STATE_FINISH ? "route finished\r\n" : "route fault: stopped\r\n");
}

static void TracingInit(void)
{
    WifiIotUartAttribute uart = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    osThreadAttr_t rxAttr = {
        .name = "MotionAckRx", .attr_bits = 0U, .cb_mem = NULL, .cb_size = 0U,
        .stack_mem = NULL, .stack_size = 3072, .priority = 27,
    };
    osThreadAttr_t routeAttr = {
        .name = "InfraredRoute", .attr_bits = 0U, .cb_mem = NULL, .cb_size = 0U,
        .stack_mem = NULL, .stack_size = 6144, .priority = 25,
    };
    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    if (UartInit(WIFI_IOT_UART_IDX_2, &uart, NULL) != 0) return;
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_GPIO_DIR_IN);
    if (osThreadNew(UartReceiveTask, NULL, &rxAttr) == NULL) return;
    (void)osThreadNew(RouteTask, NULL, &routeAttr);
}

APP_FEATURE_INIT(TracingInit);
