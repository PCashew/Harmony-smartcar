#include "stm32f10x.h"
#include "sys.h"
#include "usart.h"
#include "colorful_led.h"
#include <string.h>

int main(void)
{
    Stm32_Clock_Init(9);
    MY_NVIC_PriorityGroupConfig(2);

    uart_init(115200);

    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);

    colorful_led_Init();

    printf("READY\r\n");

    while(1)
    {
        if(USART_RX_STA == 1)
        {
            if(strcmp((char *)USART_RX_BUF, "L:0") == 0)
            {
                L_ws2812_rgb(1, WS_DARK);
                L_ws2812_rgb(2, WS_DARK);
                L_ws2812_rgb(3, WS_DARK);
                L_ws2812_rgb(4, WS_DARK);
                L_ws2812_rgb(5, WS_DARK);
                L_ws2812_rgb(6, WS_DARK);
                L_ws2812_refresh(6);

                printf("L:0\r\n");
            }
            else if(strcmp((char *)USART_RX_BUF, "L:1") == 0)
            {
                L_runingled();

                printf("L:1\r\n");
            }
            else if(strcmp((char *)USART_RX_BUF, "L:2") == 0)
            {
                L_ws2812_rgb(1, WS_WHITE);
                L_ws2812_rgb(2, WS_WHITE);
                L_ws2812_rgb(3, WS_WHITE);
                L_ws2812_rgb(4, WS_WHITE);
                L_ws2812_rgb(5, WS_WHITE);
                L_ws2812_rgb(6, WS_WHITE);
                L_ws2812_refresh(6);

                printf("L:2\r\n");
            }
            else if(strcmp((char *)USART_RX_BUF, "L:3") == 0)
            {
                L_ws2812_rgb(1, WS_RED);
                L_ws2812_rgb(2, WS_RED);
                L_ws2812_rgb(3, WS_RED);
                L_ws2812_rgb(4, WS_RED);
                L_ws2812_rgb(5, WS_RED);
                L_ws2812_rgb(6, WS_RED);
                L_ws2812_refresh(6);

                printf("L:3\r\n");
            }
            else if(strcmp((char *)USART_RX_BUF, "L:4") == 0)
            {
                L_ws2812_rgb(1, WS_GREEN);
                L_ws2812_rgb(2, WS_GREEN);
                L_ws2812_rgb(3, WS_GREEN);
                L_ws2812_rgb(4, WS_GREEN);
                L_ws2812_rgb(5, WS_GREEN);
                L_ws2812_rgb(6, WS_GREEN);
                L_ws2812_refresh(6);

                printf("L:4\r\n");
            }
            else if(strcmp((char *)USART_RX_BUF, "L:5") == 0)
            {
                L_ws2812_rgb(1, WS_BLUE);
                L_ws2812_rgb(2, WS_BLUE);
                L_ws2812_rgb(3, WS_BLUE);
                L_ws2812_rgb(4, WS_BLUE);
                L_ws2812_rgb(5, WS_BLUE);
                L_ws2812_rgb(6, WS_BLUE);
                L_ws2812_refresh(6);

                printf("L:5\r\n");
            }
            else if(strcmp((char *)USART_RX_BUF, "R:0") == 0)
            {
                R_led_CLC();

                printf("R:0\r\n");
            }
            else if(strcmp((char *)USART_RX_BUF, "R:1") == 0)
            {
                R_led_mode();

                printf("R:1\r\n");
            }
            else if(strcmp((char *)USART_RX_BUF, "R:2") == 0)
            {
                R_ws2812_rgb(1, WS_RED);
                R_ws2812_rgb(2, WS_RED);
                R_ws2812_rgb(3, WS_RED);
                R_ws2812_rgb(4, WS_RED);
                R_ws2812_rgb(5, WS_RED);
                R_ws2812_rgb(6, WS_RED);
                R_ws2812_refresh(6);

                printf("R:2\r\n");
            }
            else if(strcmp((char *)USART_RX_BUF, "R:3") == 0)
            {
                R_ws2812_rgb(1, WS_WHITE);
                R_ws2812_rgb(2, WS_WHITE);
                R_ws2812_rgb(3, WS_WHITE);
                R_ws2812_rgb(4, WS_WHITE);
                R_ws2812_rgb(5, WS_WHITE);
                R_ws2812_rgb(6, WS_WHITE);
                R_ws2812_refresh(6);

                printf("R:3\r\n");
            }
            else
            {
                printf("ERROR: %s\r\n", USART_RX_BUF);
            }

            USART_RX_STA = 0;
        }

        delay_ms(10);
    }
}