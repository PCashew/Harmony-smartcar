# Hi3861 小车桌面防跌落

本工程使用车头左右两个 TCRT5000 红外传感器检测桌面状态，并通过 Hi3861 UART2 向 STM32 发送左右轮控制指令。

## 工作流程

1. 上电后立即发送停车指令并等待 2 秒。
2. 小车保持静止，在桌面中央采样 1 秒，自动记录左右传感器的桌面电平。
3. 校准成功后倒计时 3 秒，再以较低速度前进。
4. 任一传感器出现与桌面电平不同的状态时立即停车。
5. 连续 3 次确认桌沿后，执行“后退 → 向远离桌沿的方向转向 → 停车”，随后重新检测。
6. 两侧同时检测到桌沿时，左右交替选择转向方向，避免反复困在桌角。

若传感器读取失败或启动校准结果不稳定，程序保持停车，不会盲目前进。

## 放入虚拟机

将本目录复制到 OpenHarmony 的应用目录，并建议改成纯英文目录名：

```text
applications/sample/wifi-iot/app/github_15_TableGuard
```

然后将 `applications/sample/wifi-iot/app/BUILD.gn` 的 `features` 改为：

```gn
features = [
    "github_15_TableGuard:TableGuard",
]
```

也可以参考本目录的 `APP_BUILD.gn.example`。

在 OpenHarmony 根目录编译：

```bash
cd /home/harmony/harmony/code/code-1.0
python build.py wifiiot
```

编译成功后烧录：

```text
out/wifiiot/Hi3861_wifiiot_app_allinone.bin
```

## 首次测试（必须执行）

1. 将小车架高，使车轮离开桌面，避免测试错误时直接冲出桌沿。
2. 确保两个红外传感器都对着同一块稳定桌面，再给小车上电。
3. 通过串口日志确认出现 `calibration OK` 和 `protection active`。
4. 分别用纸板模拟左侧、右侧和正前方桌沿，确认车轮动作方向正确。
5. 确认无误后再落地测试，并先在桌沿放置挡板或由人随时断电保护。

如果传感器从“桌面”移动到“悬空”时数字电平没有变化，该桌面颜色、传感器高度或比较器阈值不适合本方案；此时程序无法可靠识别桌沿，必须先调整传感器，不能直接落地测试。

## 可调参数

参数集中在 `TableGuard.c` 顶部：

- `MOTOR_SPEED_FORWARD`：正常前进速度，默认 60。
- `REVERSE_TIME_MS`：检测到桌沿后的后退时间，默认 400 ms。
- `TURN_TIME_MS`：转向时间，默认 450 ms。
- `EDGE_CONFIRM_SAMPLES`：桌沿确认次数，默认连续 3 次。

桌面颜色、反光程度、电池电量和电机差异都会影响效果。防跌落属于安全辅助功能，不能保证在所有桌面、速度和光照条件下绝不跌落；传感器必须安装在前轮之前并尽量靠近地面。
