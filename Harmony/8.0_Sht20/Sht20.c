/*
 * Copyright (c) 2020 Huawei Device Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "hal_bsp_sht20.h"
#include "hal_bsp_ssd1306.h"
#include "ohos_init.h"
#include "wifiiot_errno.h"

#define SAMPLE_PERIOD_MS 2000U
#define SAMPLE_QUEUE_DEPTH 4U
#define TASK_STACK_SIZE (4U * 1024U)

typedef struct {
    uint32_t sequence;
    uint32_t status;
    float temperature;
    float humidity;
} EnvironmentSample;

static osMessageQueueId_t g_oledQueue;
static osMessageQueueId_t g_serialQueue;
static osMutexId_t g_i2cMutex;
static bool g_oledReady;

static void DelayMilliseconds(uint32_t milliseconds)
{
    uint32_t ticks = (milliseconds * osKernelGetTickFreq() + 999U) / 1000U;
    osDelay((ticks == 0U) ? 1U : ticks);
}

static void QueueLatest(osMessageQueueId_t queue, const EnvironmentSample *sample)
{
    EnvironmentSample discarded;

    if (osMessageQueuePut(queue, sample, 0U, 0U) == osOK) {
        return;
    }

    /* Consumer lag must not block sampling: discard one stale value and retry. */
    (void)osMessageQueueGet(queue, &discarded, NULL, 0U);
    (void)osMessageQueuePut(queue, sample, 0U, 0U);
}

static void SensorTask(void *argument)
{
    EnvironmentSample sample = {0};
    (void)argument;

    while (1) {
        sample.sequence++;
        sample.temperature = 0.0F;
        sample.humidity = 0.0F;

        (void)osMutexAcquire(g_i2cMutex, osWaitForever);
        sample.status = SHT20_ReadData(&sample.temperature, &sample.humidity);
        (void)osMutexRelease(g_i2cMutex);

        QueueLatest(g_oledQueue, &sample);
        QueueLatest(g_serialQueue, &sample);
        DelayMilliseconds(SAMPLE_PERIOD_MS);
    }
}

static void OledTask(void *argument)
{
    EnvironmentSample sample;
    uint8_t line[20];
    (void)argument;

    while (1) {
        if (osMessageQueueGet(g_oledQueue, &sample, NULL, osWaitForever) != osOK) {
            continue;
        }
        if (!g_oledReady) {
            continue;
        }

        (void)osMutexAcquire(g_i2cMutex, osWaitForever);
        if (sample.status == WIFI_IOT_SUCCESS) {
            (void)snprintf((char *)line, sizeof(line), "TEMP:%7.2f C", sample.temperature);
            SSD1306_ShowStr(0, 1, line, 16);
            (void)snprintf((char *)line, sizeof(line), "HUM :%7.2f %%", sample.humidity);
            SSD1306_ShowStr(0, 2, line, 16);
        } else {
            SSD1306_ShowStr(0, 1, (uint8_t *)"SENSOR ERROR    ", 16);
            (void)snprintf((char *)line, sizeof(line), "CODE:%08X", sample.status);
            SSD1306_ShowStr(0, 2, line, 16);
        }
        (void)snprintf((char *)line, sizeof(line), "SAMPLE:%06u", sample.sequence);
        SSD1306_ShowStr(0, 3, line, 16);
        (void)osMutexRelease(g_i2cMutex);
    }
}

static void SerialUploadTask(void *argument)
{
    EnvironmentSample sample;
    (void)argument;

    while (1) {
        if (osMessageQueueGet(g_serialQueue, &sample, NULL, osWaitForever) != osOK) {
            continue;
        }

        if (sample.status == WIFI_IOT_SUCCESS) {
            /* printf is routed to the Hi3861 system debug UART. */
            printf("[SHT20_UPLOAD] seq=%u,temperature=%.2f,humidity=%.2f\r\n",
                   sample.sequence, sample.temperature, sample.humidity);
        } else {
            printf("[SHT20_UPLOAD] seq=%u,status=0x%08X\r\n",
                   sample.sequence, sample.status);
        }
    }
}

static bool CreateTask(const char *name, osThreadFunc_t entry)
{
    osThreadAttr_t attr = {0};
    attr.name = name;
    attr.stack_size = TASK_STACK_SIZE;
    attr.priority = osPriorityNormal;

    if (osThreadNew(entry, NULL, &attr) == NULL) {
        printf("Failed to create %s\r\n", name);
        return false;
    }
    return true;
}

static void Sht20OledDemo(void)
{
    uint32_t sht20Status;
    uint32_t oledStatus;
    bool tasksCreated = true;

    g_i2cMutex = osMutexNew(NULL);
    g_oledQueue = osMessageQueueNew(SAMPLE_QUEUE_DEPTH, sizeof(EnvironmentSample), NULL);
    g_serialQueue = osMessageQueueNew(SAMPLE_QUEUE_DEPTH, sizeof(EnvironmentSample), NULL);
    if ((g_i2cMutex == NULL) || (g_oledQueue == NULL) || (g_serialQueue == NULL)) {
        printf("SHT20 demo IPC initialization failed\r\n");
        return;
    }

    /* SHT20_Init configures the shared I2C0 bus before either device is used. */
    sht20Status = SHT20_Init();
    oledStatus = SSD1306_Init();
    g_oledReady = (oledStatus == WIFI_IOT_SUCCESS);

    if (g_oledReady) {
        SSD1306_CLS();
        SSD1306_ShowStr(0, 0, (uint8_t *)"SHT20 MONITOR", 16);
        SSD1306_ShowStr(0, 1, (uint8_t *)"WAITING DATA... ", 16);
    }

    printf("SHT20/OLED init: sensor=0x%08X, oled=0x%08X\r\n",
           sht20Status, oledStatus);

    tasksCreated &= CreateTask("sht20_sensor", SensorTask);
    tasksCreated &= CreateTask("sht20_oled", OledTask);
    tasksCreated &= CreateTask("sht20_serial", SerialUploadTask);
    if (!tasksCreated) {
        printf("SHT20 demo task creation incomplete\r\n");
    }
}

APP_FEATURE_INIT(Sht20OledDemo);

