#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "hi_io.h"
#include "hi_task.h"
#include "hi_time.h"
#include "ohos_init.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"

#define IR_LEFT_GPIO               13
#define IR_RIGHT_GPIO              14
#define SERVO_GPIO                  2
#define ULTRASONIC_TRIG_GPIO        7
#define ULTRASONIC_ECHO_GPIO        8

#define PROTOCOL_SIZE              10
#define PROTOCOL_HEAD_0          0xA5
#define PROTOCOL_HEAD_1          0x5A
#define PROTOCOL_TAIL            0x0D
#define CMD_STOP                 0x01
#define CMD_SET_SPEED            0x02
#define CMD_MOVE_DISTANCE        0x03
#define CMD_TURN_ANGLE           0x04
#define CMD_RETRACE_PATH         0x05
#define STATUS_ACCEPTED          0x01
#define STATUS_DONE              0x02

#define CONTROL_PERIOD_MS          20
#define EDGE_CONFIRM_SAMPLES        3
#define SAFE_CONFIRM_SAMPLES        5
#define CRUISE_SPEED_MM_S          95
#define REVERSE_CHUNK_MM           30
#define REVERSE_MAX_CHUNKS          5
#define REVERSE_SPEED_MM_S         90
#define TURN_ANGLE_X10            900
#define TURN_SPEED_MM_S            85
#define MIN_CLEARANCE_CM            25
#define FRONT_OBSTACLE_CM           32
#define SPEED_HEARTBEAT_LOOPS         5
#define RETRACE_DISTANCE_MM           80
#define RETRACE_SPEED_MM_S            70
#define RETRACE_MAX_ATTEMPTS           2
#define COMMAND_TIMEOUT_MS        8000

#define SERVO_LEFT_PULSE_US       2200
#define SERVO_LEFT_FRONT_PULSE_US 1950
#define SERVO_CENTER_PULSE_US     1650
#define SERVO_RIGHT_FRONT_PULSE_US 1350
#define SERVO_RIGHT_PULSE_US      1100
#define CRUISE_SERVO_PULSES         10
#define CRUISE_SERVO_SETTLE_MS       70
#define POST_TURN_SIDE_GUARD_SWEEPS   2
#define ECHO_TIMEOUT_US          30000ULL

typedef struct {
    WifiIotGpioValue left;
    WifiIotGpioValue right;
} InfraredState;

typedef struct {
    uint16_t left;
    uint16_t center;
    uint16_t right;
} Clearance;

static WifiIotGpioValue g_tableLeft;
static WifiIotGpioValue g_tableRight;
static volatile uint8_t g_ackCommand;
static volatile uint8_t g_ackSeq;
static volatile uint8_t g_ackStatus;
static volatile uint8_t g_ackVersion;
static volatile uint8_t g_cruiseMonitorEnabled;
static volatile uint8_t g_cruiseMonitorActive;
static volatile uint8_t g_cruiseMonitorReady;
static volatile uint8_t g_frontObstacle;
static volatile uint16_t g_frontObstacleDistance;
static volatile uint8_t g_postTurnSideGuardSweeps;
static uint8_t g_nextSeq = 1;

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
    frame[4] = (uint8_t)((uint16_t)param1 & 0xFFU);
    frame[5] = (uint8_t)(((uint16_t)param1 >> 8) & 0xFFU);
    frame[6] = (uint8_t)((uint16_t)param2 & 0xFFU);
    frame[7] = (uint8_t)(((uint16_t)param2 >> 8) & 0xFFU);
    frame[8] = Checksum(frame);
    frame[9] = PROTOCOL_TAIL;
    (void)UartWrite(WIFI_IOT_UART_IDX_2, frame, sizeof(frame));
    return seq;
}

static void EmergencyStop(void)
{
    (void)SendCommand(CMD_STOP, 0, 0);
    hi_sleep(5);
    (void)SendCommand(CMD_STOP, 0, 0);
}

static int WaitForDone(uint8_t command, uint8_t seq, unsigned int timeoutMs)
{
    unsigned int elapsed = 0;
    uint8_t seenVersion = g_ackVersion;
    while (elapsed < timeoutMs) {
        if (g_ackVersion != seenVersion) {
            seenVersion = g_ackVersion;
            if (g_ackCommand == (uint8_t)(command | 0x80U) && g_ackSeq == seq) {
                if (g_ackStatus == STATUS_DONE) return 0;
                if (g_ackStatus > STATUS_DONE) return -1;
            }
        }
        hi_sleep(10);
        elapsed += 10;
    }
    EmergencyStop();
    return -1;
}

static int RunPositionCommand(uint8_t command, int16_t value, int16_t speed)
{
    uint8_t seq = SendCommand(command, value, speed);
    return WaitForDone(command, seq, COMMAND_TIMEOUT_MS);
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
        g_ackCommand = frame[2];
        g_ackSeq = frame[3];
        g_ackStatus = frame[4];
        ++g_ackVersion;
    }
}

static int ReadInfrared(InfraredState *state)
{
    if (GpioGetInputVal(IR_LEFT_GPIO, &state->left) != 0) return -1;
    if (GpioGetInputVal(IR_RIGHT_GPIO, &state->right) != 0) return -1;
    return 0;
}

static int CalibrateTable(void)
{
    unsigned int leftHigh = 0;
    unsigned int rightHigh = 0;
    unsigned int i;
    InfraredState state;

    EmergencyStop();
    printf("PrecisionNav: place car at table center; calibrating.\r\n");
    for (i = 0; i < 50; ++i) {
        if (ReadInfrared(&state) != 0) return -1;
        if (state.left == WIFI_IOT_GPIO_VALUE1) ++leftHigh;
        if (state.right == WIFI_IOT_GPIO_VALUE1) ++rightHigh;
        hi_sleep(20);
    }
    if ((leftHigh > 10 && leftHigh < 40) || (rightHigh > 10 && rightHigh < 40)) {
        printf("PrecisionNav: unstable IR signal, calibration rejected.\r\n");
        return -1;
    }
    g_tableLeft = leftHigh >= 40 ? WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0;
    g_tableRight = rightHigh >= 40 ? WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0;
    printf("PrecisionNav: table level L=%u R=%u.\r\n",
        (unsigned int)g_tableLeft, (unsigned int)g_tableRight);
    return 0;
}

static void ServoPulse(unsigned int highUs)
{
    GpioSetOutputVal(SERVO_GPIO, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(highUs);
    GpioSetOutputVal(SERVO_GPIO, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(20000U - highUs);
}

static void ServoMove(unsigned int highUs)
{
    unsigned int i;
    for (i = 0; i < 12; ++i) ServoPulse(highUs);
    hi_sleep(80);
}

/* The low part yields to NavigationTask, so the 20 ms infrared guard keeps running. */
static void ServoMoveCruise(unsigned int highUs)
{
    unsigned int i;
    unsigned int lowMs = (20000U - highUs + 999U) / 1000U;
    for (i = 0; i < CRUISE_SERVO_PULSES; ++i) {
        GpioSetOutputVal(SERVO_GPIO, WIFI_IOT_GPIO_VALUE1);
        hi_udelay(highUs);
        GpioSetOutputVal(SERVO_GPIO, WIFI_IOT_GPIO_VALUE0);
        hi_sleep(lowMs);
    }
    hi_sleep(CRUISE_SERVO_SETTLE_MS);
}

static uint16_t MeasureDistanceCm(void)
{
    WifiIotGpioValue value;
    unsigned long long waitStart;
    unsigned long long echoStart;
    unsigned long long pulseUs;

    GpioSetOutputVal(ULTRASONIC_TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(3);
    GpioSetOutputVal(ULTRASONIC_TRIG_GPIO, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(ULTRASONIC_TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);

    waitStart = hi_get_us();
    do {
        if (GpioGetInputVal(ULTRASONIC_ECHO_GPIO, &value) != 0) return 0;
        if (hi_get_us() - waitStart > ECHO_TIMEOUT_US) return 0;
    } while (value == WIFI_IOT_GPIO_VALUE0);

    echoStart = hi_get_us();
    do {
        if (GpioGetInputVal(ULTRASONIC_ECHO_GPIO, &value) != 0) return 0;
        if (hi_get_us() - echoStart > ECHO_TIMEOUT_US) return 0;
    } while (value == WIFI_IOT_GPIO_VALUE1);

    pulseUs = hi_get_us() - echoStart;
    if (pulseUs < 100ULL || pulseUs > ECHO_TIMEOUT_US) return 0;
    return (uint16_t)((pulseUs * 17ULL) / 1000ULL);
}

static uint16_t FilteredDistanceCm(void)
{
    uint16_t samples[5];
    uint16_t value;
    uint16_t temporary;
    unsigned int valid = 0;
    unsigned int i;
    unsigned int j;

    for (i = 0; i < 7 && valid < 5; ++i) {
        value = MeasureDistanceCm();
        if (value >= 2 && value <= 400) samples[valid++] = value;
        hi_sleep(20);
    }
    if (valid < 3) return 0;
    for (i = 0; i + 1 < valid; ++i) {
        for (j = i + 1; j < valid; ++j) {
            if (samples[j] < samples[i]) {
                temporary = samples[i];
                samples[i] = samples[j];
                samples[j] = temporary;
            }
        }
    }
    return samples[valid / 2];
}

static void PauseCruiseMonitor(void)
{
    unsigned int waitLoops = 0;
    g_cruiseMonitorEnabled = 0;
    while (g_cruiseMonitorActive && waitLoops < 50U) {
        hi_sleep(10);
        ++waitLoops;
    }
    g_cruiseMonitorReady = 0;
}

static void ResumeCruiseMonitor(void)
{
    g_frontObstacle = 0;
    g_frontObstacleDistance = 0;
    g_cruiseMonitorReady = 0;
    g_cruiseMonitorEnabled = 1;
}

static void CruiseObstacleTask(void *argument)
{
    /* Ping-pong via center avoids asking the SG90 to jump directly right-to-left. */
    static const unsigned int pulseUs[4] = {
        SERVO_LEFT_FRONT_PULSE_US,
        SERVO_CENTER_PULSE_US,
        SERVO_RIGHT_FRONT_PULSE_US,
        SERVO_CENTER_PULSE_US,
    };
    unsigned int direction = 0;
    uint16_t distance;
    (void)argument;

    while (1) {
        if (!g_cruiseMonitorEnabled) {
            g_cruiseMonitorActive = 0;
            g_cruiseMonitorReady = 0;
            direction = 0;
            hi_sleep(10);
            continue;
        }

        g_cruiseMonitorActive = 1;
        ServoMoveCruise(pulseUs[direction]);
        if (!g_cruiseMonitorEnabled) {
            g_cruiseMonitorActive = 0;
            continue;
        }

        distance = MeasureDistanceCm();
        /*
         * The center sector must always be able to stop the car.  Immediately
         * after a completed avoidance turn, however, the obstacle just passed
         * commonly remains in one diagonal beam.  Ignoring only those two
         * diagonal sectors for two complete sweeps prevents repeated 90-degree
         * turns while still keeping straight-ahead protection active.
         */
        if (distance >= 2U && distance <= FRONT_OBSTACLE_CM &&
            (direction == 1U || direction == 3U ||
             g_postTurnSideGuardSweeps == 0U)) {
            g_frontObstacleDistance = distance;
            g_frontObstacle = 1;
        }

        direction = (direction + 1U) % 4U;
        if (direction == 0U && g_postTurnSideGuardSweeps > 0U) {
            --g_postTurnSideGuardSweeps;
        }
        if (!g_cruiseMonitorReady && direction == 3U) {
            g_cruiseMonitorReady = 1;
        }
        g_cruiseMonitorActive = 0;
        hi_sleep(5);
    }
}

static Clearance ScanClearance(void)
{
    Clearance result;
    ServoMove(SERVO_LEFT_PULSE_US);
    result.left = FilteredDistanceCm();
    ServoMove(SERVO_CENTER_PULSE_US);
    result.center = FilteredDistanceCm();
    ServoMove(SERVO_RIGHT_PULSE_US);
    result.right = FilteredDistanceCm();
    ServoMove(SERVO_CENTER_PULSE_US);
    printf("PrecisionNav: clearance L=%u C=%u R=%u cm.\r\n",
        result.left, result.center, result.right);
    return result;
}

static int BackUntilSafe(void)
{
    unsigned int chunk;
    unsigned int safeSamples = 0;
    InfraredState state;

    for (chunk = 0; chunk < REVERSE_MAX_CHUNKS; ++chunk) {
        if (RunPositionCommand(CMD_MOVE_DISTANCE, -REVERSE_CHUNK_MM,
                               REVERSE_SPEED_MM_S) != 0) return -1;
        safeSamples = 0;
        while (safeSamples < SAFE_CONFIRM_SAMPLES) {
            if (ReadInfrared(&state) != 0) return -1;
            if (state.left == g_tableLeft && state.right == g_tableRight) {
                ++safeSamples;
            } else {
                break;
            }
            hi_sleep(CONTROL_PERIOD_MS);
        }
        if (safeSamples >= SAFE_CONFIRM_SAMPLES) return 0;
    }
    return -1;
}

static int ChooseTurn(uint8_t leftEdge, uint8_t rightEdge, const Clearance *clearance)
{
    uint8_t leftSafe = clearance->left >= MIN_CLEARANCE_CM;
    uint8_t rightSafe = clearance->right >= MIN_CLEARANCE_CM;
    if (!leftSafe && !rightSafe) return 0;
    if (leftEdge && !rightEdge) {
        if (rightSafe) return 1;
        return leftSafe ? -1 : 0;
    }
    if (rightEdge && !leftEdge) {
        if (leftSafe) return -1;
        return rightSafe ? 1 : 0;
    }
    return clearance->left >= clearance->right ? -1 : 1;
}

static int AvoidFrontObstacle(void)
{
    Clearance clearance;
    int turn;
    unsigned int attempt;

    EmergencyStop();
    hi_sleep(40);
    for (attempt = 0; attempt <= RETRACE_MAX_ATTEMPTS; ++attempt) {
        clearance = ScanClearance();
        if (clearance.left >= MIN_CLEARANCE_CM || clearance.right >= MIN_CLEARANCE_CM) {
            if (clearance.left >= MIN_CLEARANCE_CM && clearance.right >= MIN_CLEARANCE_CM) {
                turn = clearance.left >= clearance.right ? -1 : 1;
            } else {
                turn = clearance.left >= MIN_CLEARANCE_CM ? -1 : 1;
            }
            printf("PrecisionNav: obstacle ahead, turning %s.\r\n",
                turn < 0 ? "left" : "right");
            if (RunPositionCommand(CMD_TURN_ANGLE,
                    (int16_t)(-turn * TURN_ANGLE_X10), TURN_SPEED_MM_S) != 0) {
                return -1;
            }
            g_postTurnSideGuardSweeps = POST_TURN_SIDE_GUARD_SWEEPS;
            return 0;
        }

        if (attempt >= RETRACE_MAX_ATTEMPTS) break;
        printf("PrecisionNav: both sides blocked, retracing stored path %u mm.\r\n",
            RETRACE_DISTANCE_MM);
        if (RunPositionCommand(CMD_RETRACE_PATH, RETRACE_DISTANCE_MM,
                               RETRACE_SPEED_MM_S) != 0) {
            printf("PrecisionNav: no usable stored path; stopped.\r\n");
            return -1;
        }
    }

    printf("PrecisionNav: no safe direction after route retrace; stopped.\r\n");
    return -1;
}

static int EscapeEdge(uint8_t leftEdge, uint8_t rightEdge)
{
    Clearance clearance;
    int turn;

    EmergencyStop();
    /* Leave enough time for STM32 to consume both emergency-stop frames. */
    hi_sleep(40);
    if (BackUntilSafe() != 0) {
        printf("PrecisionNav: cannot confirm safe table after reversing; locked.\r\n");
        return -1;
    }

    clearance = ScanClearance();
    turn = ChooseTurn(leftEdge, rightEdge, &clearance);
    if (turn == 0) {
        printf("PrecisionNav: neither side has reliable clearance; locked.\r\n");
        return -1;
    }

    printf("PrecisionNav: encoder turn %s 90.0 degrees.\r\n", turn < 0 ? "left" : "right");
    if (RunPositionCommand(CMD_TURN_ANGLE, (int16_t)(-turn * TURN_ANGLE_X10),
                           TURN_SPEED_MM_S) != 0) return -1;
    return 0;
}

static void NavigationTask(void *argument)
{
    InfraredState state;
    unsigned int edgeSamples = 0;
    unsigned int heartbeat = 0;
    uint8_t leftEdge = 0;
    uint8_t rightEdge = 0;
    uint8_t latchedLeft = 0;
    uint8_t latchedRight = 0;
    uint16_t obstacleDistance;
    (void)argument;

    hi_sleep(1500);
    while (CalibrateTable() != 0) {
        EmergencyStop();
        hi_sleep(1000);
    }
    ServoMove(SERVO_CENTER_PULSE_US);
    printf("PrecisionNav: protection active after 3 seconds.\r\n");
    hi_sleep(3000);
    ResumeCruiseMonitor();

    while (1) {
        if (ReadInfrared(&state) != 0) {
            EmergencyStop();
            hi_sleep(100);
            continue;
        }
        leftEdge = state.left != g_tableLeft;
        rightEdge = state.right != g_tableRight;

        if (leftEdge || rightEdge) {
            EmergencyStop();
            latchedLeft |= leftEdge;
            latchedRight |= rightEdge;
            ++edgeSamples;
            if (edgeSamples >= EDGE_CONFIRM_SAMPLES) {
                PauseCruiseMonitor();
                if (EscapeEdge(latchedLeft, latchedRight) != 0) {
                    while (1) {
                        EmergencyStop();
                        hi_sleep(500);
                    }
                }
                edgeSamples = 0;
                latchedLeft = 0;
                latchedRight = 0;
                heartbeat = 0;
                ResumeCruiseMonitor();
            }
        } else {
            edgeSamples = 0;
            latchedLeft = 0;
            latchedRight = 0;
            if (g_frontObstacle) {
                obstacleDistance = g_frontObstacleDistance;
                EmergencyStop();
                PauseCruiseMonitor();
                printf("PrecisionNav: front-sector obstacle %u cm.\r\n", obstacleDistance);
                if (AvoidFrontObstacle() != 0) {
                    /* Stay safe, but keep the navigation task recoverable. */
                    EmergencyStop();
                    heartbeat = 0;
                    ResumeCruiseMonitor();
                    hi_sleep(500);
                    continue;
                }
                heartbeat = 0;
                ResumeCruiseMonitor();
                continue;
            }

            /* After a turn, remain stopped until left-front/center/right-front were checked. */
            if (!g_cruiseMonitorReady) {
                heartbeat = 0;
            } else if (++heartbeat >= SPEED_HEARTBEAT_LOOPS) {
                heartbeat = 0;
                (void)SendCommand(CMD_SET_SPEED, CRUISE_SPEED_MM_S, CRUISE_SPEED_MM_S);
            }
        }
        hi_sleep(CONTROL_PERIOD_MS);
    }
}

static void PrecisionTableNavInit(void)
{
    WifiIotUartAttribute uart = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    osThreadAttr_t rxAttr = {
        .name = "MotionAckRx",
        .attr_bits = 0U,
        .cb_mem = NULL,
        .cb_size = 0U,
        .stack_mem = NULL,
        .stack_size = 3072,
        .priority = 27,
    };
    osThreadAttr_t navAttr = {
        .name = "PrecisionTableNav",
        .attr_bits = 0U,
        .cb_mem = NULL,
        .cb_size = 0U,
        .stack_mem = NULL,
        .stack_size = 6144,
        .priority = 25,
    };
    osThreadAttr_t obstacleAttr = {
        .name = "CruiseObstacle",
        .attr_bits = 0U,
        .cb_mem = NULL,
        .cb_size = 0U,
        .stack_mem = NULL,
        .stack_size = 3072,
        .priority = 24,
    };

    GpioInit();
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    if (UartInit(WIFI_IOT_UART_IDX_2, &uart, NULL) != 0) return;

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_2, WIFI_IOT_IO_FUNC_GPIO_2_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_7, WIFI_IOT_IO_FUNC_GPIO_7_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_8, WIFI_IOT_IO_FUNC_GPIO_8_GPIO);
    GpioSetDir(IR_LEFT_GPIO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(IR_RIGHT_GPIO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(SERVO_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(ULTRASONIC_TRIG_GPIO, WIFI_IOT_GPIO_DIR_OUT);
    GpioSetDir(ULTRASONIC_ECHO_GPIO, WIFI_IOT_GPIO_DIR_IN);
    GpioSetOutputVal(ULTRASONIC_TRIG_GPIO, WIFI_IOT_GPIO_VALUE0);
    GpioSetOutputVal(SERVO_GPIO, WIFI_IOT_GPIO_VALUE0);

    EmergencyStop();
    if (osThreadNew(UartReceiveTask, NULL, &rxAttr) == NULL) return;
    if (osThreadNew(CruiseObstacleTask, NULL, &obstacleAttr) == NULL) return;
    if (osThreadNew(NavigationTask, NULL, &navAttr) == NULL) EmergencyStop();
}

APP_FEATURE_INIT(PrecisionTableNavInit);
