# 独立蓝牙遥控烧录包

本目录使用原仓库的 `14.0_Bluetooth_control` 和 `STM32/7_串口通信` 源码编译，功能源码未修改。

2026-09-02 实机串口诊断发现原构建配置关闭了 UART2，同时开启的 AT 服务会占用 UART1，启动时因此出现 `Failed to init uart! Err code = -1`，蓝牙控制线程不会创建。当前 BIN 已使用下面的正确配置重新编译：

```text
# CONFIG_AT_SUPPORT is not set
CONFIG_UART2_SUPPORT=y
```

以后重新编译前，应在 `vendor/hisi/hi3861/hi3861/build/config/usr_config.mk` 中保持以上配置。

## 烧录顺序

1. 使用 Keil/ST-Link 将 `STM32_BluetoothControl.hex` 烧录到 STM32F103。
2. 使用 HiBurn 将 `Hi3861_BluetoothControl_allinone.bin` 烧录到 Hi3861。
3. 安装 `蓝牙控制器.apk`，连接小车的 BLE-JDY-16 模块。

## 蓝牙串口

- 波特率：9600，8N1。
- BLE TX 接 Hi3861 GPIO1（UART1 RX）。
- BLE RX 接 Hi3861 GPIO0（UART1 TX）。
- 两端必须共地；模块使用 3.3 V。

## 控制字符

- `W`：前进
- `S`：后退
- `A`：左转
- `D`：右转
- `O`：停车
- `I` / `K`：调速

原版程序没有蓝牙断连自动停车保护。首次测试必须架空车轮，断开手机连接前先发送 `O`。

旧的故障版固件保存在 `Hi3861_BluetoothControl_before_UART_fix.bin`，不要将它用于正常烧录。
