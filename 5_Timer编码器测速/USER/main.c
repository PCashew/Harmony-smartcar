#include "stm32f10x.h"
#include "sys.h"
#include "encoder.h"
#include "control_system.h"

/**************************************************************************
函数功能：初始化左右编码器，并通过USART1周期输出测速结果
入口参数：无
返回值  ：程序不退出
**************************************************************************/
int main(void)
{
    Stm32_Clock_Init(9);            // 外部8MHz晶振，系统时钟72MHz
    MY_NVIC_PriorityGroupConfig(2); // 设置中断优先级分组
    uart_init(115200);              // USART1：115200，8N1
    JTAG_Set(JTAG_SWD_DISABLE);     // 关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           // 保留SWD下载调试接口

    Encoder_Init_TIM2();            // 左轮编码器：PA0、PA1
    Encoder_Init_TIM3();            // 右轮编码器：PA6、PA7
    colorful_led_Init();            // 初始化板载炫彩灯

    OverflowTime = SPEED_SAMPLE_PERIOD_MS;
    SysTick_Config(72000000 / 1000); // SysTick每1ms触发一次中断
    printf("Encoder speed test ready\r\n");

    /* 测速和串口打印由100ms周期的SysTick中断触发 */
    while(1)
    {
        delay_ms(100);
    }
}
