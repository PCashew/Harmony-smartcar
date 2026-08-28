#ifndef HAL_BSP_SSD1306_H
#define HAL_BSP_SSD1306_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SSD1306_WIDTH 128U
#define SSD1306_HEIGHT 64U

unsigned int SSD1306_Init(void);
void SSD1306_CLS(void);
void SSD1306_ShowStr(uint8_t x, uint8_t page, const uint8_t *text, uint8_t fontSize);
void SSD1306_ShowCatFrame(uint8_t frame);

#ifdef __cplusplus
}
#endif

#endif
