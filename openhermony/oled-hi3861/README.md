# Hi3861 OLED 显示版本归档

本目录保存 2026-08-28 在 Hi3861 智能小车上完成的两版 SSD1306 OLED 程序。

## 目录说明

- `initial-text-display/`：当天最开始完成的 OLED 文字与时钟显示版本。该目录保存当时实际生成的可烧录固件；对应源码在后续动画迭代中被覆盖，因此没有将重建源码冒充原始文件。
- `final-animated-cat/`：最终确认的动态小猫版本，包括完整源码、构建配置及可烧录固件。

最终版使用 128×64 SSD1306 OLED，通过 I2C 驱动，猫咪动画共 16 帧，每帧间隔 90 ms。造型采用大头、短圆身体，脸部直接连接肩部，并包含左右爪动作。

## 烧录

使用 HiBurn 选择所需版本中的 `Hi3861_wifiiot_app_allinone.bin` 进行烧录。完成后断开 HiBurn 连接，再按开发板 `RESET` 键运行。

## 最终版源码

将 `final-animated-cat/source/7.0_I2c_Ssd1306/` 放入 OpenHarmony `applications/sample/wifi-iot/app/`，并参考 `APP_BUILD.gn.example` 把该组件加入 app 的 `features` 后执行：

```bash
python build.py wifiiot
```

## 固件校验值

校验值同时记录在 `SHA256SUMS` 中。
