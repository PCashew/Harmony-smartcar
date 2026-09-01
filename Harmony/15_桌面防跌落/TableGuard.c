#include <stdint.h>
#include <stdio.h>

#include "cmsis_os2.h"
#include "hi_io.h"
#include "hi_task.h"
#include "hi_time.h"
#include "ohos_init.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_uart.h"

#define SENSOR_LEFT_GPIO 13
#define SENSOR_RIGHT_GPIO 14

#define MOTOR_SPEED_FORWARD 60
#define MOTOR_SPEED_REVERSE 80
#define MOTOR_SPEED_TURN 90

#define CONTROL_PERIOD_MS 20
#define EDGE_CONFIRM_SAMPLES 3
#define CALIBRATION_SAMPLES 50
#define CALIBRATION_INTERVAL_MS 20
#define CALIBRATION_MIN_AGREEMENT 40

#define STOP_BEFORE_REVERSE_MS 80
#define REVERSE_TIME_MS 400
#define STOP_BEFORE_TURN_MS 80
#define TURN_TIME_MS 450
#define STOP_AFTER_TURN_MS 100

typedef struct {
    WifiIotGpioValue left;
    WifiIotGpioValue right;
} SensorState;

static uint8_t g_uartSendBuffer[6];
static WifiIotGpioValue g_tableLeftLevel;
static WifiIotGpioValue g_tableRightLevel;
static uint8_t g_turnDirection;

static void MotorControl(int leftSpeed, int rightSpeed)
{
    uint8_t leftDirection = 0;
    uint8_t rightDirection = 0;

    if (leftSpeed < 0) {
        leftDirection = 1;
        leftSpeed = -leftSpeed;
    }
    if (rightSpeed < 0) {
        rightDirection = 1;
        rightSpeed = -rightSpeed;
    }

    if (leftSpeed > 150) {
        leftSpeed = 150;
    }
    if (rightSpeed > 150) {
        rightSpeed = 150;
    }

    g_uartSendBuffer[0] = 0xFC;
    g_uartSendBuffer[1] = leftDirection;
    g_uartSendBuffer[2] = (uint8_t)leftSpeed;
    g_uartSendBuffer[3] = rightDirection;
    g_uartSendBuffer[4] = (uint8_t)rightSpeed;
    g_uartSendBuffer[5] = 0xFD;
    (void)UartWrite(WIFI_IOT_UART_IDX_2, g_uartSendBuffer, sizeof(g_uartSendBuffer));
}

static void CarStop(void)
{
    MotorControl(0, 0);
}

static void CarForward(void)
{
    MotorControl(MOTOR_SPEED_FORWARD, MOTOR_SPEED_FORWARD);
}

static void CarBackward(void)
{
    MotorControl(-MOTOR_SPEED_REVERSE, -MOTOR_SPEED_REVERSE);
}

static void CarTurnLeft(void)
{
    MotorControl(-MOTOR_SPEED_TURN, MOTOR_SPEED_TURN);
}

static void CarTurnRight(void)
{
    MotorControl(MOTOR_SPEED_TURN, -MOTOR_SPEED_TURN);
}

static int ReadSensors(SensorState *state)
{
    if (GpioGetInputVal(SENSOR_LEFT_GPIO, &state->left) != 0) {
        return -1;
    }
    if (GpioGetInputVal(SENSOR_RIGHT_GPIO, &state->right) != 0) {
        return -1;
    }
    return 0;
}

static int CalibrateTableSurface(void)
{
    unsigned int leftHighCount = 0;
    unsigned int rightHighCount = 0;
    SensorState state;

    printf("TableGuard: keep the car still in the middle of the table.\r\n");
    printf("TableGuard: calibrating surface for 1 second...\r\n");

    for (unsigned int i = 0; i < CALIBRATION_SAMPLES; ++i) {
        if (ReadSensors(&state) != 0) {
            printf("TableGuard: sensor read failed during calibration.\r\n");
            return -1;
        }
        if (state.left == WIFI_IOT_GPIO_VALUE1) {
            ++leftHighCount;
        }
        if (state.right == WIFI_IOT_GPIO_VALUE1) {
            ++rightHighCount;
        }
        hi_sleep(CALIBRATION_INTERVAL_MS);
    }

    if ((leftHighCount > CALIBRATION_SAMPLES - CALIBRATION_MIN_AGREEMENT &&
         leftHighCount < CALIBRATION_MIN_AGREEMENT) ||
        (rightHighCount > CALIBRATION_SAMPLES - CALIBRATION_MIN_AGREEMENT &&
         rightHighCount < CALIBRATION_MIN_AGREEMENT)) {
        printf("TableGuard: unstable sensor levels, calibration rejected.\r\n");
        return -1;
    }

    g_tableLeftLevel = (leftHighCount >= CALIBRATION_MIN_AGREEMENT)
        ? WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0;
    g_tableRightLevel = (rightHighCount >= CALIBRATION_MIN_AGREEMENT)
        ? WIFI_IOT_GPIO_VALUE1 : WIFI_IOT_GPIO_VALUE0;

    printf("TableGuard: calibration OK, table levels L=%u R=%u.\r\n",
        (unsigned int)g_tableLeftLevel, (unsigned int)g_tableRightLevel);
    return 0;
}

static void EscapeFromEdge(uint8_t leftEdge, uint8_t rightEdge)
{
    CarStop();
    hi_sleep(STOP_BEFORE_REVERSE_MS);

    CarBackward();
    hi_sleep(REVERSE_TIME_MS);

    CarStop();
    hi_sleep(STOP_BEFORE_TURN_MS);

    if (leftEdge && !rightEdge) {
        CarTurnRight();
        printf("TableGuard: left edge, turning right.\r\n");
    } else if (rightEdge && !leftEdge) {
        CarTurnLeft();
        printf("TableGuard: right edge, turning left.\r\n");
    } else {
        if (g_turnDirection == 0) {
            CarTurnLeft();
            g_turnDirection = 1;
            printf("TableGuard: front edge, turning left.\r\n");
        } else {
            CarTurnRight();
            g_turnDirection = 0;
            printf("TableGuard: front edge, turning right.\r\n");
        }
    }

    hi_sleep(TURN_TIME_MS);
    CarStop();
    hi_sleep(STOP_AFTER_TURN_MS);
}

static void TableGuardTask(void *argument)
{
    (void)argument;
    SensorState state;
    unsigned int edgeSampleCount = 0;
    uint8_t leftEdge = 0;
    uint8_t rightEdge = 0;

    CarStop();
    hi_sleep(2000);

    while (CalibrateTableSurface() != 0) {
        CarStop();
        printf("TableGuard: retrying calibration in 1 second.\r\n");
        hi_sleep(1000);
    }

    for (int countdown = 3; countdown > 0; --countdown) {
        CarStop();
        printf("TableGuard: starting in %d...\r\n", countdown);
        hi_sleep(1000);
    }

    printf("TableGuard: protection active.\r\n");

    while (1) {
        if (ReadSensors(&state) != 0) {
            CarStop();
            edgeSampleCount = 0;
            printf("TableGuard: sensor read failed, motors stopped.\r\n");
            hi_sleep(100);
            continue;
        }

        leftEdge = (state.left != g_tableLeftLevel);
        rightEdge = (state.right != g_tableRightLevel);

        if (!leftEdge && !rightEdge) {
            edgeSampleCount = 0;
            CarForward();
        } else {
            /* Stop on the first suspicious sample; move only after confirmation. */
            CarStop();
            ++edgeSampleCount;

            if (edgeSampleCount >= EDGE_CONFIRM_SAMPLES) {
                EscapeFromEdge(leftEdge, rightEdge);
                edgeSampleCount = 0;
            }
        }

        hi_sleep(CONTROL_PERIOD_MS);
    }
}

static void TableGuardInit(void)
{
    WifiIotUartAttribute uartAttribute = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    osThreadAttr_t threadAttribute = {
        .name = "TableGuardTask",
        .attr_bits = 0U,
        .cb_mem = NULL,
        .cb_size = 0U,
        .stack_mem = NULL,
        .stack_size = 4096,
        .priority = 25,
    };

    GpioInit();

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);
    if (UartInit(WIFI_IOT_UART_IDX_2, &uartAttribute, NULL) != 0) {
        printf("TableGuard: UART2 initialization failed.\r\n");
        return;
    }

    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13, WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14, WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    if (GpioSetDir(SENSOR_LEFT_GPIO, WIFI_IOT_GPIO_DIR_IN) != 0 ||
        GpioSetDir(SENSOR_RIGHT_GPIO, WIFI_IOT_GPIO_DIR_IN) != 0) {
        CarStop();
        printf("TableGuard: infrared GPIO initialization failed.\r\n");
        return;
    }

    CarStop();
    if (osThreadNew(TableGuardTask, NULL, &threadAttribute) == NULL) {
        CarStop();
        printf("TableGuard: failed to create control task.\r\n");
    }
}

APP_FEATURE_INIT(TableGuardInit);
