#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include "wifiiot_uart.h"
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "wifiiot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "wifiiot_pwm.h"
#include "hi_pwm.h"
#include "hi_uart.h"
#include "wifiiot_gpio_ex.h"
#include "hi_task.h"
#include "wifiiot_errno.h"

uint8_t uart_sendbuf[20];
uint8_t bluetooth_flag[1000] ;    //蓝牙标志位 

/***通信协议***/
/*
发送至stm32的数据协议
参数1：左侧电机的速度rad/s的一百倍，例如：设置转速为1rad/s则传入100
*/
void stm32motor_control(int motorA, int motorB) 
{                                               
    uint8_t A_dir = 0;
    uint8_t B_dir = 0;
    //确认旋转方向 正转：0 反转 1
    if(motorA<0){
        A_dir=1;
        motorA = -motorA;
    }else{
        A_dir=0;
    }
    if(motorB<0){
        B_dir=1;
        motorB = -motorB;
    }else{
        B_dir=0;
    }
    //限制幅度 -150 ~150
    if (motorA > 150)
    {
        motorA = 150;
    }
    if (motorB > 150)
    {
        motorB = 150;
    }

    // 数据协议 
    uart_sendbuf[0] = 0xFC;   // 帧头
    uart_sendbuf[1] = A_dir;  // 左轮方向    0正转，1反转
    uart_sendbuf[2] = motorA; // 左轮速度
    uart_sendbuf[3] = B_dir;  // 右轮方向    0正转，1反转
    uart_sendbuf[4] = motorB; // 右轮速度
    uart_sendbuf[5] = 0xFD;   // 帧尾
    UartWrite(WIFI_IOT_UART_IDX_2, (unsigned char *)uart_sendbuf, 6);
}

// 小车后退
void car_backward(void)
{
    stm32motor_control(-150, -150);
}

// 小车前进
void car_forward(void)
{
    stm32motor_control(100, 100);
}

// 小车左转
void car_left(void)
{
    stm32motor_control(-50, 150);
}

// 小车右转
void car_right(void)
{
    stm32motor_control(150, -50);
}

// 小车停止
void car_stop(void)
{
    stm32motor_control(0, 0);
}


static void car_mode_bluetooth(void)      //蓝牙模式
{
    while(1)
    {
        UartRead(WIFI_IOT_UART_IDX_1, bluetooth_flag, 1000);  
        if(bluetooth_flag[0]!=0)
        { 
            switch (bluetooth_flag[0])       //判断接收的字符
            {
            case 'O':
                car_stop();
                break;
            case 'W':
                car_forward();
                break;
            case 'A':
                car_left();
                break;
            case 'D':
                car_right();
                break;
            case 'S':
                car_backward();
                break;
            case 'I':
                stm32motor_control(100, 100);
                break;
            case 'K':
                stm32motor_control(150, 150);
                break;        
            default:
                break;
            }
            bluetooth_flag[0]=0;     //清空缓冲字符
        }
            hi_sleep(50);
        }
}


/*****任务创建*****/
static void Control(void)
{
    GpioInit();//GPIO功能初始化

    /**********************蓝牙初始化******************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_0, WIFI_IOT_IO_FUNC_GPIO_0_UART1_TXD);//GPIO_0复用为UART1_TXD
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_1, WIFI_IOT_IO_FUNC_GPIO_1_UART1_RXD);//GPIO_1复用为UART1_RX
    WifiIotUartAttribute uart_attr1 = { 
        //baud_rate: 115200
        .baudRate = 9600,
        //data_bits: 8bits
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    uint32_t ret;
    ret=UartInit(WIFI_IOT_UART_IDX_1, &uart_attr1, NULL); 
    if (ret != WIFI_IOT_SUCCESS)
    {
        printf("Failed to init uart! Err code = %d\n", ret);
        return;
    }
    printf("ble uart OK!");

    /**********************通讯串口初始化******************/
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_11, WIFI_IOT_IO_FUNC_GPIO_11_UART2_TXD);//GPIO_11复用为UART2_TXD
    IoSetFunc(WIFI_IOT_IO_NAME_GPIO_12, WIFI_IOT_IO_FUNC_GPIO_12_UART2_RXD);//GPIO_12复用为UART2_RX
    WifiIotUartAttribute uart_attr2 = {
        //波特率: 115200
        .baudRate = 115200,
        //数据位: 8bits
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    ret = UartInit(WIFI_IOT_UART_IDX_2, &uart_attr2, NULL);
    if (ret != WIFI_IOT_SUCCESS)
    {
        printf("Failed to init uart! Err code = %d\n", ret);
        return;
    }
    printf("uart OK!");

    osThreadAttr_t attr;
    attr.attr_bits = 0U;//设置osThraedJoin是否可以使用
    attr.cb_mem = NULL;//控制块指针设置
    attr.cb_size = 0U;//控制块指针大小
    attr.stack_mem = NULL;//任务栈设置
    attr.stack_size = 1024 * 4;//任务栈大小
    //创建任务1
    attr.name = "car_mode_bluetooth";//创建任务名称
    attr.priority = 25;//任务优先级
    if (osThreadNew((osThreadFunc_t)car_mode_bluetooth, NULL, &attr) == NULL)
    {
        printf("Falied to create car_mode_bluetooth!\n");
    }
}
APP_FEATURE_INIT(Control);//启动任务