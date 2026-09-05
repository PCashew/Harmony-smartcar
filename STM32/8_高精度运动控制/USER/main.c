#include "stm32f10x.h"
#include "sys.h"

int main(void)
{
    u8 command;
    u8 seq;
    u8 event_command;
    u8 event_seq;
    u8 event_status;
    s16 param1;
    s16 param2;
    s16 telemetry1;
    s16 telemetry2;
    MotionStatus result;

    Stm32_Clock_Init(9);
    MY_NVIC_PriorityGroupConfig(2);
    uart_init(115200);
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();
    PWM_Init(7199, 9);
    colorful_led_Init();
    Motion_Init();
    SysTick_Config(72000000 / 1000);

    while (1) {
        if (Protocol_GetFrame(&command, &seq, &param1, &param2)) {
            switch (command) {
            case CMD_STOP:
                result = Motion_Stop(seq);
                break;
            case CMD_SET_SPEED:
                result = Motion_SetSpeed(param1, param2, seq);
                break;
            case CMD_MOVE_DISTANCE:
                result = Motion_MoveDistance(param1, (u16)param2, seq);
                break;
            case CMD_TURN_ANGLE:
                result = Motion_TurnAngle(param1, (u16)param2, seq);
                break;
            case CMD_RETRACE_PATH:
                result = Motion_RetracePath((u16)param1, (u16)param2, seq);
                break;
            case CMD_GET_ENCODERS:
                Motion_GetEncoderMm(&telemetry1, &telemetry2);
                Protocol_SendValues(command, seq, telemetry1, telemetry2);
                continue;
            default:
                result = MOTION_STATUS_BAD_PARAM;
                break;
            }
            Protocol_SendStatus(command, seq, (u8)result);
        }

        if (Motion_GetEvent(&event_command, &event_seq, &event_status)) {
            Protocol_SendStatus(event_command, event_seq, event_status);
        }
    }
}
