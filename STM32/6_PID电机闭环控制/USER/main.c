#include "stm32f10x.h"
#include "sys.h"
#include "control_system.h"

/**************************************************************************
函数功能：周期执行 PID 闭环前进和后退
入口参数：无
返回值  ：程序持续循环，不返回
说明    ：两轮 PI 调速并结合累计编码器差，自动修正左右轮速度
**************************************************************************/
int main(void)
{
    Stm32_Clock_Init(9);              // 外部 8MHz 晶振，系统时钟 72MHz
    MY_NVIC_PriorityGroupConfig(2);   // 设置中断优先级分组
    uart_init(115200);                // 串口输出当前运动阶段
    JTAG_Set(JTAG_SWD_DISABLE);       // 关闭 JTAG，释放相关 IO
    JTAG_Set(SWD_ENABLE);             // 保留 SWD 下载调试接口

    Control_System_Init();            // 初始化 TIM4 PWM、TIM2/TIM3 编码器
    colorful_led_Init();              // 初始化前后两组 WS2812 灯带
    Control_Stop();
    LED_All_Off_Step();

    printf("PID forward/reverse cycle ready\r\n");
    delay_ms(3000);                   // 留出放置小车和松开复位键的时间

   /* while (1)
    {
        
        printf("ACTION: PID forward\r\n");
        Control_RunTimedPID(1, 1, CAR_STRAIGHT_TIME_MS,
                            LED_Off_Step);

       
        printf("ACTION: PID reverse\r\n");
        Control_RunTimedPID(-1, -1, CAR_REVERSE_TIME_MS,
                            R_led_CLC);
    }
    */
   while (1)
{
    distance = Ultrasonic_GetDistance();

    printf("Distance: %.1f cm\r\n", distance);

    delay_ms(200);
}
}
