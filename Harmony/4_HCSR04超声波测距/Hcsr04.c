#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_watchdog.h"
#include "hi_io.h"
#include "hi_time.h"

// HC-SR04：Trig -> GPIO7，Echo -> GPIO8
#define GPIO_8      8
#define GPIO_7      7
#define GPIO_FUNC   0

#define HC_SR04_TIMEOUT_US   30000
#define HC_SR04_MIN_CM       2.0f
#define HC_SR04_MAX_CM       400.0f

#define SAMPLE_COUNT         5

#define IoTGpioSetDir GpioSetDir


static float GetDistanceOnce(void)
{
    unsigned long start_time = 0;
    unsigned long echo_time = 0;
    unsigned long timeout_start = 0;

    WifiIotGpioValue value = WIFI_IOT_GPIO_VALUE0;

    float distance = -1.0f;

    // 配置 GPIO 功能
    hi_io_set_func(GPIO_8, GPIO_FUNC);
    hi_io_set_func(GPIO_7, GPIO_FUNC);

    // Echo 输入
    GpioSetDir(GPIO_8, WIFI_IOT_GPIO_DIR_IN);

    // Trig 输出
    GpioSetDir(GPIO_7, WIFI_IOT_GPIO_DIR_OUT);

    // 先确保 Trig 为低电平
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE0);
    hi_udelay(2);

    // 发送至少 10us 的高电平触发脉冲
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE1);
    hi_udelay(20);
    GpioSetOutputVal(GPIO_7, WIFI_IOT_GPIO_VALUE0);

    /*
     * 等待 Echo 上升沿
     */
    timeout_start = hi_get_us();

    while (1) {
        GpioGetInputVal(GPIO_8, &value);

        if (value == WIFI_IOT_GPIO_VALUE1) {
            start_time = hi_get_us();
            break;
        }

        // 超时，没有收到 Echo
        if ((hi_get_us() - timeout_start) > HC_SR04_TIMEOUT_US) {
            return -1.0f;
        }
    }

    /*
     * 等待 Echo 下降沿
     */
    while (1) {
        GpioGetInputVal(GPIO_8, &value);

        if (value == WIFI_IOT_GPIO_VALUE0) {
            echo_time = hi_get_us() - start_time;
            break;
        }

        // Echo 高电平时间过长
        if ((hi_get_us() - start_time) > HC_SR04_TIMEOUT_US) {
            return -1.0f;
        }
    }

    /*
     * 声速约 343 m/s
     * = 0.0343 cm/us
     *
     * 超声波往返一次：
     *
     * distance = time * 0.0343 / 2
     */
    distance = echo_time * 0.0343f / 2.0f;

    // 过滤明显超出 HC-SR04 工作范围的值
    if (distance < HC_SR04_MIN_CM ||
        distance > HC_SR04_MAX_CM) {
        return -1.0f;
    }

    return distance;
}


/*
 * 简单排序
 */
static void SortFloat(float *data, int len)
{
    int i;
    int j;

    float temp;

    for (i = 0; i < len - 1; i++) {
        for (j = 0; j < len - 1 - i; j++) {

            if (data[j] > data[j + 1]) {

                temp = data[j];
                data[j] = data[j + 1];
                data[j + 1] = temp;
            }
        }
    }
}


/*
 * HC-SR04 多次测量 + 中值滤波
 */
float GetDistance(void)
{
    float samples[SAMPLE_COUNT];

    int valid_count = 0;
    int try_count = 0;

    float distance;

    /*
     * 最多尝试 10 次，
     * 收集 5 个有效数据
     */
    while (valid_count < SAMPLE_COUNT &&
           try_count < 10) {

        distance = GetDistanceOnce();

        if (distance > 0.0f) {

            samples[valid_count] = distance;
            valid_count++;
        }

        try_count++;

        /*
         * HC-SR04 相邻两次测量最好留一点间隔，
         * 防止上一次超声波回波干扰下一次
         */
        osDelay(60);
    }

    // 没有有效数据
    if (valid_count == 0) {
        return -1.0f;
    }

    // 排序
    SortFloat(samples, valid_count);

    /*
     * 中值滤波
     *
     * 例如：
     * 29.8
     * 30.1
     * 30.2
     * 30.5
     * 96.3
     *
     * 中值为 30.2
     */
    if (valid_count % 2 == 1) {

        return samples[valid_count / 2];

    } else {

        return (samples[valid_count / 2 - 1] +
                samples[valid_count / 2]) / 2.0f;
    }
}


/*
 * HC-SR04 测距线程
 */
void Hcsrtxt(void *parame)
{
    float distance;

    (void)parame;

    printf("start test hcsr04\r\n");

    while (1) {

        distance = GetDistance();

        if (distance > 0.0f) {

            printf("distance = %.1f cm\r\n", distance);

        } else {

            printf("HC-SR04 measure failed!\r\n");
        }

        /*
         * GetDistance() 内部已经测了多次，
         * 所以这里不用太频繁
         */
        osDelay(200);
    }
}


/* 任务入口 */
static void Hcsr04(void)
{
    WatchDogDisable();

    osThreadAttr_t attr;

    attr.name = "Hcsr04";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 10240;
    attr.priority = osPriorityNormal;

    if (osThreadNew(Hcsrtxt, NULL, &attr) == NULL) {

        printf("Failed to create Task!\n");
    }
}


APP_FEATURE_INIT(Hcsr04);