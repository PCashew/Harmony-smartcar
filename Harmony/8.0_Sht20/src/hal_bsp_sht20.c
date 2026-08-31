#include "hal_bsp_sht20.h"

#include <stddef.h>
#include <stdio.h>
#include <unistd.h>

#include "wifiiot_errno.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_i2c.h"
#include "wifiiot_i2c_ex.h"

#define SHT20_NO_HOLD_TEMP_CMD 0xF3
#define SHT20_NO_HOLD_HUMI_CMD 0xF5
#define SHT20_SOFT_RESET_CMD 0xFE
#define SHT20_TEMP_DELAY_US (85U * 1000U)
#define SHT20_HUMI_DELAY_US (50U * 1000U)

static uint32_t SHT20_RecvData(uint8_t *data, size_t size)
{
    WifiIotI2cData i2cData = {0};
    i2cData.receiveBuf = data;
    i2cData.receiveLen = size;
    return I2cRead(SHT20_I2C_IDX, SHT20_I2C_ADDR, &i2cData);
}

static uint32_t SHT20_WriteCommand(uint8_t command)
{
    uint8_t buffer[] = {command};
    WifiIotI2cData i2cData = {0};
    i2cData.sendBuf = buffer;
    i2cData.sendLen = sizeof(buffer);
    return I2cWrite(SHT20_I2C_IDX, SHT20_I2C_ADDR, &i2cData);
}

/* SHT20 CRC-8: polynomial x^8 + x^5 + x^4 + 1 (0x31), initial value 0. */
static uint8_t SHT20_CalculateCrc(const uint8_t *data, size_t size)
{
    uint8_t crc = 0;
    size_t i;
    uint8_t bit;

    for (i = 0; i < size; ++i) {
        crc ^= data[i];
        for (bit = 0; bit < 8U; ++bit) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x31U)
                                : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

static uint32_t SHT20_ReadRaw(uint8_t command, useconds_t delayUs, uint16_t *rawValue)
{
    uint8_t buffer[3] = {0};
    uint32_t result;

    result = SHT20_WriteCommand(command);
    if (result != WIFI_IOT_SUCCESS) {
        printf("SHT20 command 0x%02X failed: 0x%08X\r\n", command, result);
        return result;
    }

    usleep(delayUs);
    result = SHT20_RecvData(buffer, sizeof(buffer));
    if (result != WIFI_IOT_SUCCESS) {
        printf("SHT20 read for command 0x%02X failed: 0x%08X\r\n", command, result);
        return result;
    }

    if (SHT20_CalculateCrc(buffer, 2U) != buffer[2]) {
        printf("SHT20 CRC mismatch for command 0x%02X\r\n", command);
        return (uint32_t)WIFI_IOT_FAILURE;
    }

    /* Bits 1..0 are status bits and are not part of the measurement value. */
    *rawValue = (uint16_t)((((uint16_t)buffer[0] << 8U) | buffer[1]) & 0xFFFCU);
    return WIFI_IOT_SUCCESS;
}

uint32_t SHT20_ReadData(float *temp, float *humi)
{
    uint16_t rawTemperature;
    uint16_t rawHumidity;
    uint32_t result;
    float humidity;

    if ((temp == NULL) || (humi == NULL)) {
        return (uint32_t)WIFI_IOT_FAILURE;
    }

    result = SHT20_ReadRaw(SHT20_NO_HOLD_TEMP_CMD, SHT20_TEMP_DELAY_US,
                           &rawTemperature);
    if (result != WIFI_IOT_SUCCESS) {
        return result;
    }

    result = SHT20_ReadRaw(SHT20_NO_HOLD_HUMI_CMD, SHT20_HUMI_DELAY_US,
                           &rawHumidity);
    if (result != WIFI_IOT_SUCCESS) {
        return result;
    }

    *temp = -46.85F + (175.72F * (float)rawTemperature / 65536.0F);
    humidity = -6.0F + (125.0F * (float)rawHumidity / 65536.0F);
    if (humidity < 0.0F) {
        humidity = 0.0F;
    } else if (humidity > 100.0F) {
        humidity = 100.0F;
    }
    *humi = humidity;
    return WIFI_IOT_SUCCESS;
}

uint32_t SHT20_Init(void)
{
    uint32_t result;

    GpioInit();
    (void)IoSetFunc(WIFI_IOT_IO_NAME_GPIO_10, WIFI_IOT_IO_FUNC_GPIO_10_I2C0_SDA);
    (void)IoSetFunc(WIFI_IOT_IO_NAME_GPIO_9, WIFI_IOT_IO_FUNC_GPIO_9_I2C0_SCL);

    result = I2cInit(SHT20_I2C_IDX, SHT20_I2C_SPEED);
    if (result != WIFI_IOT_SUCCESS) {
        printf("I2C0 initialization failed: 0x%08X\r\n", result);
        return result;
    }

    result = I2cSetBaudrate(SHT20_I2C_IDX, SHT20_I2C_SPEED);
    if (result != WIFI_IOT_SUCCESS) {
        printf("I2C0 baudrate setup failed: 0x%08X\r\n", result);
        return result;
    }

    result = SHT20_WriteCommand(SHT20_SOFT_RESET_CMD);
    if (result != WIFI_IOT_SUCCESS) {
        printf("SHT20 reset failed: 0x%08X\r\n", result);
        return result;
    }

    usleep(100U * 1000U);
    printf("SHT20 initialization succeeded\r\n");
    return WIFI_IOT_SUCCESS;
}

