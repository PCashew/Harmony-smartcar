#include "hal_bsp_ssd1306.h"
#include "hal_bsp_ssd1306_fonts.h"
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>
#include "wifiiot_errno.h"
#include "wifiiot_i2c.h"

static uint32_t SSD1306_SendData(uint8_t *data, size_t size)
{
  WifiIotI2cData i2cData = {0};
  i2cData.sendBuf = data;
  i2cData.sendLen = size;

  return I2cWrite(SSD1306_I2C_IDX, SSD1306_I2C_ADDR, &i2cData);
}

// 写命令
static uint32_t SSD1306_WriteCmd(uint8_t byte)
{
  uint8_t buffer[] = {0x00, byte};
  return SSD1306_SendData(buffer, sizeof(buffer));
}
// 写数据
static uint32_t SSD1306_WiteData(uint8_t byte)
{
  uint8_t buffer[] = {0x40, byte};
  return SSD1306_SendData(buffer, sizeof(buffer));
}

uint32_t SSD1306_Init(void)
{
    static const uint8_t initCommands[] = {
        0xAE, 0x20, 0x10, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0xFF, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0xF0, 0xD9, 0x22, 0xDA, 0x12, 0xDB,
        0x20, 0x8D, 0x14, 0xAF
    };
    uint32_t result;
    size_t i;

    /* I2C0 pins and baudrate are configured once by SHT20_Init. */
    usleep(100U * 1000U);
    for (i = 0; i < sizeof(initCommands); ++i) {
        result = SSD1306_WriteCmd(initCommands[i]);
        if (result != WIFI_IOT_SUCCESS) {
            printf("SSD1306 command 0x%02X failed: 0x%08X\r\n",
                   initCommands[i], result);
            return result;
        }
    }

    SSD1306_SetPos(0, 0);
    printf("SSD1306 initialization succeeded\r\n");
    return WIFI_IOT_SUCCESS;
}
/**
 * @brief  垂直滚动函数
 * @note   
 * @retval None
 */
void OLED_Set_Vertical_Rol(void)
{
	SSD1306_WriteCmd(0x2e);		// 停止滚动

	SSD1306_WriteCmd(0xa3);		// 
	SSD1306_WriteCmd(0x00);
	SSD1306_WriteCmd(0x40);

	SSD1306_WriteCmd(0x2a);
	SSD1306_WriteCmd(0x00);
	SSD1306_WriteCmd(0x00);
	SSD1306_WriteCmd(0x01);
	SSD1306_WriteCmd(0x00);
	SSD1306_WriteCmd(0x04);

	SSD1306_WriteCmd(0x2f);
}

void SSD1306_SetPos(uint8_t x, uint8_t y) //设置起始点坐标
{
  SSD1306_WriteCmd(0xb0 + y);
  SSD1306_WriteCmd(((x & 0xf0) >> 4) | 0x10);
  SSD1306_WriteCmd((x & 0x0f) | 0x01);
}

void SSD1306_Fill(uint8_t fill_Data) //全屏填充
{
  unsigned char m, n;
  for (m = 0; m < 8; m++)
  {
    SSD1306_WriteCmd(0xb0 + m); //page0-page1
    SSD1306_WriteCmd(0x00);     //low column start address
    SSD1306_WriteCmd(0x10);     //high column start address
    for (n = 0; n < 128; n++)
    {
      SSD1306_WiteData(fill_Data);
    }
  }
}

void SSD1306_CLS(void) //清屏
{
  SSD1306_Fill(0x00);
}

//--------------------------------------------------------------
// Prototype      : void OLED_ON(void)
// Calls          :
// Parameters     : none
// Description    : 将OLED从休眠中唤醒
//--------------------------------------------------------------
void SSD1306_ON(void)
{
  SSD1306_WriteCmd(0X8D); //设置电荷泵
  SSD1306_WriteCmd(0X14); //开启电荷泵
  SSD1306_WriteCmd(0XAF); //OLED唤醒
}

//--------------------------------------------------------------
// Prototype      : void OLED_OFF(void)
// Calls          :
// Parameters     : none
// Description    : 让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
//--------------------------------------------------------------
void SSD1306_OFF(void)
{
  SSD1306_WriteCmd(0X8D); //设置电荷泵
  SSD1306_WriteCmd(0X10); //关闭电荷泵
  SSD1306_WriteCmd(0XAE); //OLED休眠
}

/**
 * @brief  显示字符串
 * @note   
 * @param  x: 
 * @param  y: 
 * @param  ch[]: 
 * @param  TextSize: 
 * @retval None
 */
void SSD1306_ShowStr(uint8_t x, uint8_t y, uint8_t ch[], uint8_t TextSize)
{
  unsigned char c = 0, i = 0, j = 0;
  switch (TextSize)
  {
  case 8:
  {
    while (ch[j] != '\0')
    {
      c = ch[j] - 32;
      if (x > 126)
      {
        x = 0;
        y++;
      }
      SSD1306_SetPos(x, y);
      for (i = 0; i < 6; i++)
        SSD1306_WiteData(F6x8[c][i]);
      x += 6;
      j++;
    }
  }
  break;
  case 16:
  {
    y *= 2;
    while (ch[j] != '\0')
    {
      c = ch[j] - 32;
      if (x > 120)
      {
        x = 0;
        y++;
      }
      SSD1306_SetPos(x, y);
      for (i = 0; i < 8; i++)
        SSD1306_WiteData(F8X16[c * 16 + i]);
      SSD1306_SetPos(x, y + 1);
      for (i = 0; i < 8; i++)
        SSD1306_WiteData(F8X16[c * 16 + i + 8]);
      x += 8;
      j++;
    }
  }
  break;
  }
}

/**
 * @brief  显示图片
 * @note   
 * @param  x0: 
 * @param  y0: 
 * @param  x1: 
 * @param  y1: 
 * @param  BMP[]: 
 * @retval None
 */
void SSD1306_DrawBMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,uint8_t BMP[])
{ 	
 unsigned int j=0;
 unsigned char x,y;
  
  if(y1%8==0) y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++)
	{
		SSD1306_SetPos(x0,y);
    for(x=x0;x<x1;x++)
	    {      
	    	SSD1306_WiteData(BMP[j++]);	    	
	    }
	}
} 

