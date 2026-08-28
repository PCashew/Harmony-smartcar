#include <stdio.h>
#include <unistd.h>

#include "cmsis_os2.h"
#include "hal_bsp_ssd1306.h"
#include "ohos_init.h"

static void OledTask(void *argument)
{
    uint8_t frame = 0U;
    unsigned int result;

    (void)argument;
    result = SSD1306_Init();
    if (result != 0U) {
        printf("I2C SSD1306 init failed: %u\r\n", result);
        return;
    }

    printf("I2C SSD1306 init succeeded\r\n");
    while (1) {
        SSD1306_ShowCatFrame(frame++);
        usleep(90000);
    }
}

static void I2cSsd1306Demo(void)
{
    osThreadAttr_t options = {0};

    options.name = "oled_task";
    options.stack_size = 2048U;
    options.priority = osPriorityNormal;
    if (osThreadNew(OledTask, NULL, &options) == NULL) {
        printf("Failed to create OLED task\r\n");
    }
}

APP_FEATURE_INIT(I2cSsd1306Demo);
