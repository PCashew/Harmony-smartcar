#include "stm32f10x.h"
#include "sys.h"

/* Kept local because this example intentionally does not add another header. */
u8 UART_TryReadByte(u8 *data);

typedef enum {
  LED_MODE_OFF,
  LED_MODE_LEFT_RUNNING,
  LED_MODE_RIGHT_RUNNING,
  LED_MODE_CIRCLE,
  LED_MODE_COLORFUL
} LED_Mode;

LED_Mode led_mode = LED_MODE_OFF;

void USART_Command_Process(void) {
  u8 command;

  if (UART_TryReadByte(&command) == 0)
    return;

  switch (command) {
  case '0':
    led_mode = LED_MODE_OFF;
    printf("MODE: OFF\r\n");
    break;

  case '1':
    led_mode = LED_MODE_LEFT_RUNNING;
    printf("MODE: LEFT RUNNING\r\n");
    break;

  case '2':
    led_mode = LED_MODE_RIGHT_RUNNING;
    printf("MODE: RIGHT RUNNING\r\n");
    break;

  case '3':
    led_mode = LED_MODE_CIRCLE;
    printf("MODE: CIRCLE\r\n");
    break;

  case '4':
    led_mode = LED_MODE_COLORFUL;
    printf("MODE: COLORFUL\r\n");
    break;

  default:
    printf("ERROR: UNKNOWN COMMAND '%c'\r\n", command);
    break;
  }
}

int main(void) {
  /* Clock, UART, and RGB LED initialization */
  Stm32_Clock_Init(9);            // 外部时钟8Mhz 9倍频  8*9= 72mhz倍频72mhz
  MY_NVIC_PriorityGroupConfig(2); //=====中断优先级分组
  uart_init(115200);              //=====串口初始化为115200
  JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
  JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
  colorful_led_Init();            //=====炫彩灯初始化

  /* Show Command Bar */
  printf("+---------------+----------------------------+\r\n");
  printf("| Command       | LED Mode                   \r\n");
  printf("+---------------+----------------------------+\r\n");
  printf("| 0             | LED_MODE_OFF               \r\n");
  printf("| 1             | LED_MODE_LEFT_RUNNING      \r\n");
  printf("| 2             | LED_MODE_RIGHT_RUNNING     \r\n");
  printf("| 3             | LED_MODE_CIRCLE            \r\n");
  printf("| 4             | LED_MODE_COLORFUL          \r\n");
  printf("+---------------+----------------------------+\r\n");
  printf("Please enter a command (1-4, or '0' to quit): ");

  /* Main Program Startup */
  while (1) {
    USART_Command_Process();

    switch (led_mode) {
    case LED_MODE_OFF:
      LED_Off_Step();
      break;

    case LED_MODE_LEFT_RUNNING:
      Left_Running_Step();
      break;

    case LED_MODE_RIGHT_RUNNING:
      Right_Running_Step();
      break;

    case LED_MODE_CIRCLE:
      Circle_Running_Step();
      break;

    case LED_MODE_COLORFUL:
      Colorful_Step();
      break;
    }
  }
}
