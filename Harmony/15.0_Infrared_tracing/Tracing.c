#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include "wifiiot_uart.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_pwm.h"
#include "hi_pwm.h"
#include "hi_uart.h"
#include "wifiiot_gpio_ex.h"
#include "hi_task.h"


//查阅机器人板原理图可知
//左边的红外传感器与3861芯片的GPIO13连接
//右边的红外传感器与3861芯片的GPIO14连接
#define GPIOL 13
#define GPIOR 14
#define GPIO_FUNC 0

#define CONTROL_PERIOD_MS 30
#define CORRECTION_CONFIRM_SAMPLES 2
#define STRONG_CORRECTION_SAMPLES 6
#define EVENT_CONFIRM_SAMPLES 4
#define CLEAR_CONFIRM_SAMPLES 3

#define JUNCTION_TURN_MIN_SAMPLES 10
#define JUNCTION_TURN_MAX_SAMPLES 35
#define JUNCTION_CENTER_SAMPLES 3
#define JUNCTION_EDGE_CONFIRM_SAMPLES 2
#define RECENTER_HOLD_SAMPLES 0
#define MIN_STAGE_RUN_SAMPLES 20
#define FINISH_SECOND_BAR_MAX_SAMPLES 200

#define FORWARD_SPEED 40
#define CORRECTION_INNER_SPEED 36
#define CORRECTION_OUTER_SPEED 50
#define STRONG_CORRECTION_INNER_SPEED 25
#define STRONG_CORRECTION_OUTER_SPEED 65
#define JUNCTION_INNER_SPEED 30
#define JUNCTION_OUTER_SPEED 75

uint8_t uart_sendbuf[20];

WifiIotGpioValue io_status_left;
WifiIotGpioValue io_status_right;

/***通信协议***/
/*
发送至stm32的数据协议
参数：电机A/B速度rad/s的一百倍，例如设置转速为1rad/s则传入100
*/
void stm32motor_control(int motorA, int motorB)
{
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;
    //确认旋转方向 正转：0 反转 1
    if(motorA<0){
        A_dir=1;
        motorA = -motorA;
    }else{
        A_dir=0;
    }
    if(motorB<0){
        B_dir=1;
        motorB = -motorB;
    }else{
        B_dir=0;
    }
    //限制幅度 -150 ~150
    if (motorA > 150)
    {
        motorA = 150;
    }
    if (motorB > 150)
    {
        motorB = 150;
    }

    // 数据协议
    uart_sendbuf[0] = 0xFC;   // 帧头
    uart_sendbuf[1] = A_dir;  // 电机A方向    0正转，1反转
    uart_sendbuf[2] = motorA; // 电机A速度
    uart_sendbuf[3] = B_dir;  // 电机B方向    0正转，1反转
    uart_sendbuf[4] = motorB; // 电机B速度
    uart_sendbuf[5] = 0xFD;   // 帧尾
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

// 小车后退
void car_backward(void)
{
    stm32motor_control(-FORWARD_SPEED, -FORWARD_SPEED);
}

// 小车前进
void car_forward(void)
{
    stm32motor_control(FORWARD_SPEED, FORWARD_SPEED);
}

// 小车左转 循迹使用
void car_left_tra(void)
{
    stm32motor_control(CORRECTION_OUTER_SPEED, CORRECTION_INNER_SPEED);
}

// 小车右转 循迹使用
void car_right_tra(void)
{
    stm32motor_control(CORRECTION_INNER_SPEED, CORRECTION_OUTER_SPEED);
}

static void car_left_strong(void)
{
    stm32motor_control(STRONG_CORRECTION_OUTER_SPEED,
                       STRONG_CORRECTION_INNER_SPEED);
}

static void car_right_strong(void)
{
    stm32motor_control(STRONG_CORRECTION_INNER_SPEED,
                       STRONG_CORRECTION_OUTER_SPEED);
}

static void car_left_junction(void)
{
    stm32motor_control(JUNCTION_OUTER_SPEED, JUNCTION_INNER_SPEED);
}

static void car_right_junction(void)
{
    stm32motor_control(JUNCTION_INNER_SPEED, JUNCTION_OUTER_SPEED);
}

// 小车停止
void car_stop(void)
{
    stm32motor_control(0, 0);
}

typedef enum {
    FOLLOW_FORWARD,
    FOLLOW_LEFT,
    FOLLOW_RIGHT,
    FOLLOW_SPECIAL
} FollowAction;

typedef enum {
    ROUTE_FIRST_JUNCTION,
    ROUTE_SECOND_JUNCTION,
    ROUTE_FINISH,
    ROUTE_DONE
} RouteStage;

static FollowAction last_correction = FOLLOW_FORWARD;
static unsigned int correction_hold = 0;

static FollowAction classify_sensors(WifiIotGpioValue left, WifiIotGpioValue right)
{
    // LOW means black tape; gray floor and white tape are both HIGH.
    // The narrow black line runs between the probes while centered.
    if (left == WIFI_IOT_GPIO_VALUE0 && right == WIFI_IOT_GPIO_VALUE0) {
        return FOLLOW_SPECIAL;
    }
    if (right == WIFI_IOT_GPIO_VALUE0) {
        return FOLLOW_RIGHT;
    }
    if (left == WIFI_IOT_GPIO_VALUE0) {
        return FOLLOW_LEFT;
    }
    return FOLLOW_FORWARD;
}

static FollowAction confirm_lane_action(FollowAction action,
                                        FollowAction *candidate,
                                        unsigned int *samples)
{
    if (action != FOLLOW_LEFT && action != FOLLOW_RIGHT) {
        *candidate = FOLLOW_FORWARD;
        *samples = 0;
        return action;
    }

    if (action != *candidate) {
        *candidate = action;
        *samples = 1;
        return FOLLOW_FORWARD;
    }

    if (*samples < STRONG_CORRECTION_SAMPLES) {
        (*samples)++;
    }
    return *samples >= CORRECTION_CONFIRM_SAMPLES ? action : FOLLOW_FORWARD;
}

static int route_logic_self_check(void)
{
    FollowAction candidate = FOLLOW_FORWARD;
    unsigned int samples = 0;
    int filter_ok =
        confirm_lane_action(FOLLOW_LEFT, &candidate, &samples) == FOLLOW_FORWARD &&
        confirm_lane_action(FOLLOW_LEFT, &candidate, &samples) == FOLLOW_LEFT;

    for (unsigned int i = samples; i < STRONG_CORRECTION_SAMPLES; i++) {
        confirm_lane_action(FOLLOW_LEFT, &candidate, &samples);
    }

    return classify_sensors(WIFI_IOT_GPIO_VALUE1, WIFI_IOT_GPIO_VALUE1) == FOLLOW_FORWARD &&
           classify_sensors(WIFI_IOT_GPIO_VALUE0, WIFI_IOT_GPIO_VALUE1) == FOLLOW_LEFT &&
           classify_sensors(WIFI_IOT_GPIO_VALUE1, WIFI_IOT_GPIO_VALUE0) == FOLLOW_RIGHT &&
           classify_sensors(WIFI_IOT_GPIO_VALUE0, WIFI_IOT_GPIO_VALUE0) == FOLLOW_SPECIAL &&
           filter_ok && samples == STRONG_CORRECTION_SAMPLES;
}

static FollowAction read_sensors(void)
{
    GpioGetInputVal(GPIOL,&io_status_left); //获取GPIO13引脚的输入电平值
    GpioGetInputVal(GPIOR,&io_status_right);//获取GPIO14引脚的输入电平值
    return classify_sensors(io_status_left, io_status_right);
}

static int take_junction(int turn_left)
{
    unsigned int centered = 0;
    unsigned int edge_count = 0;

    // ponytail: two sensors can reacquire the selected branch but cannot
    // measure heading; keep the timeout and use encoders if this ceiling matters.
    for (unsigned int sample = 0; sample < JUNCTION_TURN_MAX_SAMPLES; sample++) {
        if (turn_left) {
            car_left_junction();
        } else {
            car_right_junction();
        }
        hi_sleep(CONTROL_PERIOD_MS);

        FollowAction action = read_sensors();
        if ((turn_left && action == FOLLOW_LEFT) ||
            (!turn_left && action == FOLLOW_RIGHT)) {
            if (edge_count < JUNCTION_EDGE_CONFIRM_SAMPLES) {
                edge_count++;
            }
        } else if (action != FOLLOW_FORWARD) {
            edge_count = 0;
        }

        if (sample >= JUNCTION_TURN_MIN_SAMPLES &&
            edge_count >= JUNCTION_EDGE_CONFIRM_SAMPLES &&
            action == FOLLOW_FORWARD) {
            if (++centered >= JUNCTION_CENTER_SAMPLES) {
                return 1;
            }
        } else {
            centered = 0;
        }
    }

    return 0;
}

static void follow_lane(FollowAction action, unsigned int sustained_samples)
{
    if (action == FOLLOW_LEFT || action == FOLLOW_RIGHT) {
        last_correction = action;
        correction_hold = RECENTER_HOLD_SAMPLES;
    }

    if (action == FOLLOW_LEFT) {
        if (sustained_samples >= STRONG_CORRECTION_SAMPLES) {
            car_left_strong();
        } else {
            car_left_tra();
        }
    } else if (action == FOLLOW_RIGHT) {
        if (sustained_samples >= STRONG_CORRECTION_SAMPLES) {
            car_right_strong();
        } else {
            car_right_tra();
        }
    } else if (last_correction == FOLLOW_LEFT && correction_hold > 0) {
        correction_hold--;
        car_left_tra();
    } else if (last_correction == FOLLOW_RIGHT && correction_hold > 0) {
        correction_hold--;
        car_right_tra();
    } else {
        // ponytail: white/white is both centered and fully lost with two binary
        // probes. Drive straight; extra hardware is required to distinguish it.
        last_correction = FOLLOW_FORWARD;
        car_forward();
    }
}



/***循迹函数****/
void trace_module(void)
{
    RouteStage stage = ROUTE_FIRST_JUNCTION;
    unsigned int clear_count = 0;
    unsigned int special_count = 0;
    unsigned int stage_run_samples = 0;
    unsigned int finish_bar_count = 0;
    unsigned int finish_gap_samples = 0;
    FollowAction correction_candidate = FOLLOW_FORWARD;
    unsigned int correction_samples = 0;
    int armed = 0;

    if (!route_logic_self_check()) {
        car_stop();
        printf("route logic self-check failed\r\n");
        return;
    }

    printf("route start: leave the left start area\r\n");

    while (stage != ROUTE_DONE) {
        FollowAction action = read_sensors();

        if (finish_bar_count > 0 &&
            ++finish_gap_samples > FINISH_SECOND_BAR_MAX_SAMPLES) {
            finish_bar_count = 0;
            finish_gap_samples = 0;
            printf("finish marker first bar expired\r\n");
        }

        if (!armed) {
            car_forward();

            if (action == FOLLOW_FORWARD) {
                if (++clear_count >= CLEAR_CONFIRM_SAMPLES) {
                    armed = 1;
                    clear_count = 0;
                    stage_run_samples = 0;
                    printf("route armed: stage=%d\r\n", stage);
                }
            } else {
                clear_count = 0;
            }
            hi_sleep(CONTROL_PERIOD_MS);
            continue;
        }

        if (action != FOLLOW_SPECIAL) {
            special_count = 0;
            if (stage_run_samples < 0xffffffffU) {
                stage_run_samples++;
            }
            follow_lane(confirm_lane_action(action, &correction_candidate,
                                            &correction_samples),
                        correction_samples);
            hi_sleep(CONTROL_PERIOD_MS);
            continue;
        }

        correction_candidate = FOLLOW_FORWARD;
        correction_samples = 0;

        if (stage_run_samples < MIN_STAGE_RUN_SAMPLES &&
            !(stage == ROUTE_FINISH && finish_bar_count == 1)) {
            special_count = 0;
            car_forward();
            hi_sleep(CONTROL_PERIOD_MS);
            continue;
        }

        if (++special_count < EVENT_CONFIRM_SAMPLES) {
            car_forward();
            hi_sleep(CONTROL_PERIOD_MS);
            continue;
        }

        special_count = 0;
        armed = 0;
        last_correction = FOLLOW_FORWARD;
        correction_hold = 0;
        stage_run_samples = 0;

        if (stage == ROUTE_FIRST_JUNCTION) {
            printf("junction 1: turn left\r\n");
            if (take_junction(1)) {
                printf("junction 1: branch acquired\r\n");
                stage = ROUTE_SECOND_JUNCTION;
            } else {
                printf("junction 1: turn timeout, stage unchanged\r\n");
            }
        } else if (stage == ROUTE_SECOND_JUNCTION) {
            printf("junction 2: turn right\r\n");
            if (take_junction(0)) {
                printf("junction 2: branch acquired\r\n");
                stage = ROUTE_FINISH;
            } else {
                printf("junction 2: turn timeout, stage unchanged\r\n");
            }
        } else if (finish_bar_count == 0) {
            finish_bar_count = 1;
            finish_gap_samples = 0;
            printf("finish marker first bar\r\n");
        } else {
            car_stop();
            printf("finish marker second bar reached\r\n");
            stage = ROUTE_DONE;
        }
    }

    car_stop();
}


/*****任务创建*****/
static void Tracing(void)
{
    GpioInit();//GPIO功能初始化

    /**********************通讯串口初始化******************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);//GPIO_11复用为UART2_TXD
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);//GPIO_12复用为UART2_RX

        WifiIotUartAttribute uart_attr2 = {
        //波特率: 115200
        .baudRate = 115200,
        //数据位: 8bits
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
        };
    UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);

    /**********************红外初始化******************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_13,WIFI_IOT_IO_FUNC_GPIO_13_GPIO);
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_14,WIFI_IOT_IO_FUNC_GPIO_14_GPIO);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_13,WIFI_IOT_GPIO_DIR_IN);
    GpioSetDir(WIFI_IOT_IO_NAME_GPIO_14,WIFI_IOT_GPIO_DIR_IN);

    osThreadAttr_t attr;
    attr.attr_bits = 0U;//设置osThraedJoin是否可以使用
    attr.cb_mem = NULL;//控制块指针设置
    attr.cb_size = 0U;//控制块指针大小
    attr.stack_mem = NULL;//任务栈设置
    attr.stack_size = 1024 * 4;//任务栈大小
    //创建任务1
    attr.name = "trace_module";//创建任务名称
    attr.priority = 25;//任务优先级
    if (osThreadNew((osThreadFunc_t)trace_module, NULL, &attr) == NULL)
    {
        printf("Falied to create trace_module!\n");
    }
}

APP_FEATURE_INIT(Tracing);//启动任务
