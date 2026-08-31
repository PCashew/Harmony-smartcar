#include "stm32f10x.h"
#include "sys.h"
#include "control_system.h"

#define CAR_ACTION_TIME_MS ((u16)3200)
#define CAR_STOP_TIME_MS ((u16)800)
#define LED_FRAME_TIME_MS ((u16)80)

typedef void (*LED_EffectStep)(void);

/**************************************************************************
函数功能：同步执行指定的电机动作和逐帧灯效，动作结束后停车熄灯
入口参数：左右轮速度、灯效函数和持续时间
返回值  ：无
**************************************************************************/
static void RunTimedAction(s16 left_speed, s16 right_speed,
                           LED_EffectStep led_effect, u16 run_time) {
  u16 elapsed = 0;

  Control_SetWheels(left_speed, right_speed);
  while (elapsed < run_time) {
    led_effect();
    delay_ms(LED_FRAME_TIME_MS);
    elapsed += LED_FRAME_TIME_MS;
  }

  Control_Stop();
  LED_All_Off_Step();
  delay_ms(CAR_STOP_TIME_MS);
}

/**************************************************************************
函数功能：按时间循环演示六种小车运动及其对应灯效
入口参数：无
返回值  ：程序不退出
**************************************************************************/
int main(void) {
  Stm32_Clock_Init(9);            // 外部8MHz晶振，系统时钟72MHz
  MY_NVIC_PriorityGroupConfig(2); // 设置中断优先级分组
  JTAG_Set(JTAG_SWD_DISABLE);     // 关闭JTAG接口
  JTAG_Set(SWD_ENABLE);           // 保留SWD下载调试接口

  Control_System_Init(); // 初始化TIM4电机PWM
  colorful_led_Init();   // 初始化前后WS2812灯带
  Control_Stop();
  LED_All_Off_Step();
  delay_ms(3000); // 留出放置小车和松开复位键的时间

  while (1) {
    /* 单轮右转：左轮前进，红色顺时针跑马灯 */
    RunTimedAction(CAR_TEST_SPEED, 0, LED_Red_Clockwise_Step,
                   CAR_ACTION_TIME_MS);

    /* 单轮左转：右轮前进，蓝色逆时针跑马灯 */
    RunTimedAction(0, CAR_TEST_SPEED, LED_Blue_CounterClockwise_Step,
                   CAR_ACTION_TIME_MS);

    /* 直行：左右轮前进，环形炫彩灯 */
    RunTimedAction(CAR_TEST_SPEED, CAR_TEST_SPEED, LED_Colorful_Ring_Step,
                   CAR_ACTION_TIME_MS);

    /* 倒退：左右轮反转，全车红色呼吸灯 */
    RunTimedAction(-CAR_TEST_SPEED, -CAR_TEST_SPEED,
                   LED_Red_Breathing_Step, CAR_ACTION_TIME_MS);

    /* 双轮旋转右转：左前右后，红色顺时针渐变跑马灯 */
    RunTimedAction(CAR_TEST_SPEED, -CAR_TEST_SPEED,
                   LED_Red_Clockwise_Gradient_Step, CAR_ACTION_TIME_MS);

    /* 双轮旋转左转：左后右前，蓝色逆时针渐变跑马灯 */
    RunTimedAction(-CAR_TEST_SPEED, CAR_TEST_SPEED,
                   LED_Blue_CounterClockwise_Gradient_Step,
                   CAR_ACTION_TIME_MS);
  }
}
