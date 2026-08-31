#include "control_system.h"

#include "sys.h"

/* 左电机：PB14/L-IB控制方向，PB7/L-IA（TIM4_CH2）输出PWM */
#define LEFT_DIRECTION PBout(14)
#define LEFT_PWM TIM4->CCR2

/* 右电机：PB13/R-IA控制方向，PB6/R-IB（TIM4_CH1）输出PWM */
#define RIGHT_DIRECTION PBout(13)
#define RIGHT_PWM TIM4->CCR1

/* 左编码器PA0/PA1，右编码器PA6/PA7 */
#define ENCODER_SAMPLE_COUNT ((u16)10000)
#define ENCODER_SAMPLE_INTERVAL_US ((u16)100)

static u8 ReadLeftEncoderState(void) {
  u16 input = GPIO_ReadInputData(GPIOA);
  u8 state = 0;

  if (input & GPIO_Pin_0)
    state |= 0x01;
  if (input & GPIO_Pin_1)
    state |= 0x02;
  return state;
}

static u8 ReadRightEncoderState(void) {
  u16 input = GPIO_ReadInputData(GPIOA);
  return (u8)((input >> 6) & 0x03);
}

/**************************************************************************
函数功能：限制电机速度，防止PWM比较值超出自动重装载值
入口参数：speed，范围可正可负
返回值  ：限制到[-CAR_PWM_PERIOD, CAR_PWM_PERIOD]后的速度
**************************************************************************/
static s16 ClampSpeed(s16 speed) {
  if (speed > (s16)CAR_PWM_PERIOD)
    return (s16)CAR_PWM_PERIOD;
  if (speed < -(s16)CAR_PWM_PERIOD)
    return -(s16)CAR_PWM_PERIOD;
  return speed;
}

/**************************************************************************
函数功能：设置左轮速度和方向
入口参数：speed，正数正转，负数反转，0停止
返回值  ：无
**************************************************************************/
static void SetLeftMotor(s16 speed) {
  speed = ClampSpeed(speed);

  if (speed >= 0) {
    LEFT_DIRECTION = 0;
    LEFT_PWM = (u16)speed;
  } else {
    LEFT_DIRECTION = 1;
    LEFT_PWM = CAR_PWM_PERIOD - (u16)(-speed);
  }
}

/**************************************************************************
函数功能：设置右轮速度和方向
入口参数：speed，正数正转，负数反转，0停止
返回值  ：无
**************************************************************************/
static void SetRightMotor(s16 speed) {
  speed = ClampSpeed(speed);

  if (speed >= 0) {
    RIGHT_DIRECTION = 0;
    RIGHT_PWM = (u16)speed;
  } else {
    RIGHT_DIRECTION = 1;
    RIGHT_PWM = CAR_PWM_PERIOD - (u16)(-speed);
  }
}

/**************************************************************************
函数功能：同时设置左右轮速度
入口参数：left_speed左轮速度，right_speed右轮速度
返回值  ：无
**************************************************************************/
void Control_SetWheels(s16 left_speed, s16 right_speed) {
  SetLeftMotor(left_speed);
  SetRightMotor(right_speed);
}

/**************************************************************************
函数功能：停止左右两个电机
入口参数：无
返回值  ：无
**************************************************************************/
void Control_Stop(void) { Control_SetWheels(0, 0); }

/**************************************************************************
函数功能：初始化电机方向GPIO和TIM4双通道PWM
入口参数：无
返回值  ：无
**************************************************************************/
void Control_System_Init(void) {
  GPIO_InitTypeDef gpio;
  TIM_TimeBaseInitTypeDef timer;
  TIM_OCInitTypeDef output_compare;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,
                         ENABLE); // 使能编码器和电机GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  // 使能TIM4时钟

  /* PB13、PB14配置为电机方向推挽输出 */
  gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  gpio.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &gpio);

  /* PA0、PA1、PA6、PA7配置为编码器上拉输入 */
  gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
  gpio.GPIO_Mode = GPIO_Mode_IPU;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio);

  /* PB6、PB7分别对应TIM4_CH1、TIM4_CH2 */
  gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  gpio.GPIO_Mode = GPIO_Mode_AF_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &gpio);

  TIM_TimeBaseStructInit(&timer);
  timer.TIM_Period = CAR_PWM_PERIOD;
  timer.TIM_Prescaler = CAR_PWM_PRESCALER;
  timer.TIM_ClockDivision = TIM_CKD_DIV1;
  timer.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &timer);

  TIM_OCStructInit(&output_compare);
  output_compare.TIM_OCMode = TIM_OCMode_PWM1;
  output_compare.TIM_OutputState = TIM_OutputState_Enable;
  output_compare.TIM_Pulse = 0;
  output_compare.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM4, &output_compare);
  TIM_OC2Init(TIM4, &output_compare);

  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM4, ENABLE);

  Control_Stop();       // 定时器启动前先确保两轮停止
  TIM_Cmd(TIM4, ENABLE);
}

/**************************************************************************
函数功能：依次驱动左右电机并统计编码器状态变化次数
入口参数：left_edges、right_edges，保存左右编码器边沿计数
返回值  ：无；测试结束后两个电机均停止
**************************************************************************/
void Control_RunSelfTest(u16 *left_edges, u16 *right_edges) {
  u16 sample;
  u8 previous_state;
  u8 current_state;

  if (left_edges == 0 || right_edges == 0) {
    Control_Stop();
    return;
  }

  *left_edges = 0;
  *right_edges = 0;
  Control_Stop();
  delay_ms(300);

  previous_state = ReadLeftEncoderState();
  Control_SetWheels(CAR_TEST_SPEED, 0);
  for (sample = 0; sample < ENCODER_SAMPLE_COUNT; sample++) {
    current_state = ReadLeftEncoderState();
    if (current_state != previous_state) {
      (*left_edges)++;
      previous_state = current_state;
    }
    delay_us(ENCODER_SAMPLE_INTERVAL_US);
  }

  Control_Stop();
  delay_ms(300);

  previous_state = ReadRightEncoderState();
  Control_SetWheels(0, CAR_TEST_SPEED);
  for (sample = 0; sample < ENCODER_SAMPLE_COUNT; sample++) {
    current_state = ReadRightEncoderState();
    if (current_state != previous_state) {
      (*right_edges)++;
      previous_state = current_state;
    }
    delay_us(ENCODER_SAMPLE_INTERVAL_US);
  }

  Control_Stop();
}

/**************************************************************************
函数功能：解析一个串口控制指令并立即更新左右轮状态
入口参数：command，0~8或S/F/B/L/R
返回值  ：1表示有效指令，0表示非法指令且电机已停止
**************************************************************************/
u8 Control_ExecuteCommand(u8 command) {
  switch (command) {
  case '0':
  case 'S':
  case 's':
    Control_Stop();
    break;

  case '1':
    Control_SetWheels(CAR_DEFAULT_SPEED, 0);
    break;
  case '2':
    Control_SetWheels(-CAR_DEFAULT_SPEED, 0);
    break;
  case '3':
    Control_SetWheels(0, CAR_DEFAULT_SPEED);
    break;
  case '4':
    Control_SetWheels(0, -CAR_DEFAULT_SPEED);
    break;

  case '5':
  case 'F':
  case 'f':
    Control_SetWheels(CAR_DEFAULT_SPEED, CAR_DEFAULT_SPEED);
    break;
  case '6':
  case 'B':
  case 'b':
    Control_SetWheels(-CAR_DEFAULT_SPEED, -CAR_DEFAULT_SPEED);
    break;
  case '7':
  case 'L':
  case 'l':
    Control_SetWheels(-CAR_DEFAULT_SPEED, CAR_DEFAULT_SPEED);
    break;
  case '8':
  case 'R':
  case 'r':
    Control_SetWheels(CAR_DEFAULT_SPEED, -CAR_DEFAULT_SPEED);
    break;

  default:
    Control_Stop(); // 未知指令执行安全停车
    return 0;
  }

  return 1;
}
