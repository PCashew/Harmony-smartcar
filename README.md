# 智能小车项目合集

本仓库整理了智能小车课程中的 STM32 与 Harmony/OpenHarmony 代码。项目按平台划分为两个独立板块，便于查找、学习和维护。

## 目录结构

```text
.
├── STM32/                         # STM32F10x + Keil 工程
│   ├── 1_工程模板_跑马灯/
│   ├── 2_串口收发打印/
│   ├── 4_PWM驱动电机/
│   ├── 5_Timer编码器测速/
│   └── 6_PID电机闭环控制/
└── Harmony/                       # Harmony/OpenHarmony 模块
    ├── 3_SG90舵机互斥/
    ├── 4_HCSR04超声波测距/
    ├── 8.0_Sht20/
    ├── harmony_share/
    │   └── paho_mqtt/
    └── oled-hi3861/                # OLED 显示示例与固件
├── 学习日志/                       # 课程学习记录
├── README.md
└── .gitignore
```

## STM32

STM32 板块包含基于 STM32F10x 标准外设库和 Keil MDK 的控制端工程。

| 项目 | 内容 |
| --- | --- |
| [1_工程模板_跑马灯](STM32/1_工程模板_跑马灯/) | 基础工程模板与跑马灯示例 |
| [2_串口收发打印](STM32/2_串口收发打印/) | USART 串口收发与调试输出 |
| [4_PWM驱动电机](STM32/4_PWM驱动电机/) | 使用 PWM 驱动直流电机 |
| [5_Timer编码器测速](STM32/5_Timer编码器测速/) | 定时器与编码器测速 |
| [6_PID电机闭环控制](STM32/6_PID电机闭环控制/) | 电机 PID 闭环控制 |

工程入口通常位于各项目的 `USER` 目录，可使用 Keil 打开其中的 `.uvprojx` 工程文件。

## Harmony

Harmony 板块包含 OpenHarmony 设备端功能模块与共享组件。

| 项目 | 内容 |
| --- | --- |
| [3_SG90舵机互斥](Harmony/3_SG90舵机互斥/) | SG90 舵机控制与互斥处理 |
| [4_HCSR04超声波测距](Harmony/4_HCSR04超声波测距/) | HCSR04 超声波测距与 Tick 计时 |
| [8.0_Sht20](Harmony/8.0_Sht20/) | SHT20 温湿度采集与 SSD1306 OLED 显示 |
| [harmony_share/paho_mqtt](Harmony/harmony_share/paho_mqtt/) | Paho MQTT 通信组件 |
| [oled-hi3861](Harmony/oled-hi3861/) | Hi3861 OLED 文字与动画显示示例 |

SG90、HCSR04 与 SHT20 目录包含 `BUILD.gn`，应作为 Harmony/OpenHarmony 构建模块使用。

## 学习日志

课程学习过程与实验记录统一保存在 [学习日志](学习日志/) 目录中。

## 开发环境

- STM32：Keil MDK、STM32F10x 标准外设库、ST-Link
- Harmony/OpenHarmony：OpenHarmony 构建环境、GN、MQTT

## 使用说明

1. 根据开发平台进入 `STM32` 或 `Harmony` 目录。
2. STM32 工程通过对应的 `.uvprojx` 文件打开。
3. Harmony 模块根据目标 OpenHarmony 工程的组件配置接入，并通过 `BUILD.gn` 参与构建。
