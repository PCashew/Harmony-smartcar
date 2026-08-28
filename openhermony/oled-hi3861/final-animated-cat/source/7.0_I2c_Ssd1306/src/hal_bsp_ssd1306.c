#include "hal_bsp_ssd1306.h"

#include <stddef.h>
#include <string.h>

#include "hal_bsp_ssd1306_fonts.h"
#include "wifiiot_gpio.h"
#include "wifiiot_gpio_ex.h"
#include "wifiiot_i2c.h"

#define SSD1306_I2C_IDX WIFI_IOT_I2C_IDX_0
#define SSD1306_I2C_BAUDRATE 400000U
#define SSD1306_I2C_WRITE_ADDRESS 0x78U
#define SSD1306_CONTROL_COMMAND 0x00U
#define SSD1306_CONTROL_DATA 0x40U
#define SSD1306_PAGE_COUNT (SSD1306_HEIGHT / 8U)

static uint8_t g_frameBuffer[SSD1306_WIDTH * SSD1306_PAGE_COUNT];

static unsigned int Ssd1306Write(const uint8_t *buffer, unsigned int length)
{
    WifiIotI2cData data = {0};
    data.sendBuf = (unsigned char *)buffer;
    data.sendLen = length;
    return I2cWrite(SSD1306_I2C_IDX, SSD1306_I2C_WRITE_ADDRESS, &data);
}

static unsigned int Ssd1306WriteCommand(uint8_t command)
{
    uint8_t packet[2] = {SSD1306_CONTROL_COMMAND, command};
    return Ssd1306Write(packet, sizeof(packet));
}

static unsigned int Ssd1306Flush(void)
{
    uint8_t packet[SSD1306_WIDTH + 1U];
    unsigned int result = 0;
    uint8_t page;

    packet[0] = SSD1306_CONTROL_DATA;
    for (page = 0; page < SSD1306_PAGE_COUNT; ++page) {
        result |= Ssd1306WriteCommand((uint8_t)(0xB0U + page));
        result |= Ssd1306WriteCommand(0x00U);
        result |= Ssd1306WriteCommand(0x10U);
        (void)memcpy(&packet[1], &g_frameBuffer[page * SSD1306_WIDTH], SSD1306_WIDTH);
        result |= Ssd1306Write(packet, sizeof(packet));
    }
    return result;
}

static void Ssd1306SetPixel(uint8_t x, uint8_t y, uint8_t enabled)
{
    uint16_t index;
    uint8_t mask;

    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    index = (uint16_t)x + ((uint16_t)(y / 8U) * SSD1306_WIDTH);
    mask = (uint8_t)(1U << (y % 8U));
    if (enabled != 0U) {
        g_frameBuffer[index] |= mask;
    } else {
        g_frameBuffer[index] &= (uint8_t)~mask;
    }
}

static void Ssd1306DrawLineColor(int x0, int y0, int x1, int y1, uint8_t enabled)
{
    int dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy = (y0 < y1) ? (y0 - y1) : (y1 - y0);
    int sy = (y0 < y1) ? 1 : -1;
    int error = dx + dy;
    int errorTwice;

    while (1) {
        Ssd1306SetPixel((uint8_t)x0, (uint8_t)y0, enabled);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        errorTwice = error * 2;
        if (errorTwice >= dy) {
            error += dy;
            x0 += sx;
        }
        if (errorTwice <= dx) {
            error += dx;
            y0 += sy;
        }
    }
}

static void Ssd1306DrawThickLineColor(int x0, int y0, int x1, int y1, uint8_t enabled)
{
    int dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
    int dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);

    Ssd1306DrawLineColor(x0, y0, x1, y1, enabled);
    if (dx >= dy) {
        Ssd1306DrawLineColor(x0, y0 - 1, x1, y1 - 1, enabled);
        Ssd1306DrawLineColor(x0, y0 + 1, x1, y1 + 1, enabled);
    } else {
        Ssd1306DrawLineColor(x0 - 1, y0, x1 - 1, y1, enabled);
        Ssd1306DrawLineColor(x0 + 1, y0, x1 + 1, y1, enabled);
    }
}

static const uint8_t *Ssd1306FindGlyph(char character)
{
    size_t index;
    static const uint8_t unknown[5] = {0x02, 0x01, 0x51, 0x09, 0x06};

    if (character >= 'a' && character <= 'z') {
        character = (char)(character - ('a' - 'A'));
    }
    for (index = 0; index < SSD1306_FONT_GLYPH_COUNT; ++index) {
        if (g_ssd1306Font[index].character == character) {
            return g_ssd1306Font[index].columns;
        }
    }
    return unknown;
}

static void Ssd1306DrawCharacter(uint8_t x, uint8_t y, char character, uint8_t fontSize)
{
    const uint8_t *glyph = Ssd1306FindGlyph(character);
    uint8_t scale = (fontSize >= 16U) ? 2U : 1U;
    uint8_t column;
    uint8_t row;
    uint8_t sx;
    uint8_t sy;

    for (column = 0; column < 6U; ++column) {
        for (row = 0; row < 8U; ++row) {
            uint8_t pixel = 0U;
            if (column < 5U && row < 7U) {
                pixel = (uint8_t)((glyph[column] >> row) & 0x01U);
            }
            for (sx = 0; sx < scale; ++sx) {
                for (sy = 0; sy < scale; ++sy) {
                    Ssd1306SetPixel((uint8_t)(x + column * scale + sx),
                        (uint8_t)(y + row * scale + sy), pixel);
                }
            }
        }
    }
}

unsigned int SSD1306_Init(void)
{
    static const uint8_t initCommands[] = {
        0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
        0x40, 0x8D, 0x14, 0xAF
    };
    unsigned int result;
    size_t index;

    GpioInit();
    result = IoSetFunc(WIFI_IOT_IO_NAME_GPIO_9, WIFI_IOT_IO_FUNC_GPIO_9_I2C0_SCL);
    result |= IoSetFunc(WIFI_IOT_IO_NAME_GPIO_10, WIFI_IOT_IO_FUNC_GPIO_10_I2C0_SDA);
    result |= I2cInit(SSD1306_I2C_IDX, SSD1306_I2C_BAUDRATE);
    if (result != 0U) {
        return result;
    }

    for (index = 0; index < sizeof(initCommands); ++index) {
        result = Ssd1306WriteCommand(initCommands[index]);
        if (result != 0U) {
            return result;
        }
    }
    SSD1306_CLS();
    return 0U;
}

void SSD1306_CLS(void)
{
    (void)memset(g_frameBuffer, 0, sizeof(g_frameBuffer));
    (void)Ssd1306Flush();
}

void SSD1306_ShowStr(uint8_t x, uint8_t page, const uint8_t *text, uint8_t fontSize)
{
    uint8_t cursor = x;
    uint8_t y = (uint8_t)(page * 8U);
    uint8_t scale = (fontSize >= 16U) ? 2U : 1U;
    uint8_t advance = (uint8_t)(6U * scale);

    if (text == NULL || page >= SSD1306_PAGE_COUNT) {
        return;
    }
    while (*text != '\0' && cursor < SSD1306_WIDTH) {
        Ssd1306DrawCharacter(cursor, y, (char)*text, fontSize);
        if ((uint16_t)cursor + advance >= SSD1306_WIDTH) {
            break;
        }
        cursor = (uint8_t)(cursor + advance);
        ++text;
    }
    (void)Ssd1306Flush();
}

void SSD1306_ShowCatFrame(uint8_t frame)
{
    static const int8_t bodyOffset[16] = {
        -2, -2, -1, -1, 0, 1, 1, 2, 2, 2, 1, 1, 0, -1, -1, -2
    };
    static const int8_t fanHandX[16] = {
        5, 7, 9, 11, 13, 15, 13, 11, 9, 7, 5, 4, 5, 7, 9, 11
    };
    static const int8_t fanHandY[16] = {
        38, 36, 34, 32, 31, 32, 34, 36, 38, 40, 38, 36, 34, 33, 34, 36
    };
    static const int8_t bobOffset[16] = {
        0, 0, -1, -1, -2, -1, 0, 0, 0, 0, -1, -1, -2, -1, 0, 0
    };
    uint8_t animationFrame = (uint8_t)(frame % 16U);
    int centerX = 64 + bodyOffset[animationFrame];
    int handX = centerX + fanHandX[animationFrame];
    int bob = bobOffset[animationFrame];
    int handY = fanHandY[animationFrame] + bob;
    int tailLift = (animationFrame < 8U) ? 0 : -4;

    (void)memset(g_frameBuffer, 0, sizeof(g_frameBuffer));

    /* Oversized head with pointed ears and broad cheeks. */
    Ssd1306DrawThickLineColor(centerX - 29, 27 + bob, centerX - 24, 21 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 24, 21 + bob, centerX - 22, 7 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 22, 7 + bob, centerX - 12, 14 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 12, 14 + bob, centerX, 12 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX, 12 + bob, centerX + 12, 14 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 12, 14 + bob, centerX + 22, 7 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 22, 7 + bob, centerX + 24, 21 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 24, 21 + bob, centerX + 29, 27 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 29, 27 + bob, centerX + 24, 32 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 24, 32 + bob, centerX + 15, 35 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 15, 35 + bob, centerX + 10, 36 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 10, 36 + bob, centerX - 15, 35 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 15, 35 + bob, centerX - 24, 32 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 24, 32 + bob, centerX - 29, 27 + bob, 1U);

    /* Cheeks flow directly into a short rounded body, without a neck column. */
    Ssd1306DrawThickLineColor(centerX - 10, 36 + bob, centerX - 18, 41 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 18, 41 + bob, centerX - 20, 47 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 20, 47 + bob, centerX - 19, 57 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 19, 57 + bob, centerX - 12, 60 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 12, 60 + bob, centerX - 3, 58 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 3, 58 + bob, centerX + 3, 58 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 3, 58 + bob, centerX + 12, 60 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 12, 60 + bob, centerX + 19, 57 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 19, 57 + bob, centerX + 20, 47 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 20, 47 + bob, centerX + 18, 41 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 18, 41 + bob, centerX + 10, 36 + bob, 1U);

    /* Long eyes and tiny w-shaped mouth. */
    Ssd1306DrawThickLineColor(centerX - 11, 20 + bob, centerX - 11, 28 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 11, 20 + bob, centerX + 11, 28 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 3, 30 + bob, centerX, 32 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX, 32 + bob, centerX + 3, 30 + bob, 1U);

    /* Tail swings behind the compact body. */
    Ssd1306DrawThickLineColor(centerX + 18, 50 + bob,
        centerX + 27, 48 + bob + tailLift, 1U);
    Ssd1306DrawThickLineColor(centerX + 27, 48 + bob + tailLift,
        centerX + 33, 43 + bob + tailLift, 1U);

    /* Left paw covers the nose; right rounded paw fans beside the face. */
    Ssd1306DrawThickLineColor(centerX - 17, 49 + bob, centerX - 15, 42 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 15, 42 + bob, centerX - 11, 35 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 11, 35 + bob, centerX - 5, 29 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX - 5, 29 + bob, centerX, 31 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 17, 49 + bob, centerX + 14, 44 + bob, 1U);
    Ssd1306DrawThickLineColor(centerX + 14, 44 + bob, handX, handY, 1U);
    Ssd1306DrawThickLineColor(handX - 3, handY, handX - 2, handY - 2, 1U);
    Ssd1306DrawThickLineColor(handX - 2, handY - 2, handX + 1, handY - 3, 1U);
    Ssd1306DrawThickLineColor(handX + 1, handY - 3, handX + 3, handY - 1, 1U);
    Ssd1306DrawThickLineColor(handX + 3, handY - 1, handX + 2, handY + 2, 1U);
    Ssd1306DrawThickLineColor(handX + 2, handY + 2, handX, handY + 3, 1U);

    (void)Ssd1306Flush();
}
