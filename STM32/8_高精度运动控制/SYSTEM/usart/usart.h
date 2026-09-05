#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"

#define PROTOCOL_FRAME_SIZE 10U
#define PROTOCOL_HEAD_0     0xA5U
#define PROTOCOL_HEAD_1     0x5AU
#define PROTOCOL_TAIL       0x0DU

#define CMD_STOP            0x01U
#define CMD_SET_SPEED       0x02U
#define CMD_MOVE_DISTANCE   0x03U
#define CMD_TURN_ANGLE      0x04U
#define CMD_RETRACE_PATH    0x05U
#define CMD_GET_ENCODERS    0x07U

void uart_init(u32 bound);
u8 Protocol_GetFrame(u8 *command, u8 *seq, s16 *param1, s16 *param2);
void Protocol_SendStatus(u8 command, u8 seq, u8 status);
void Protocol_SendValues(u8 command, u8 seq, s16 value1, s16 value2);

#endif
