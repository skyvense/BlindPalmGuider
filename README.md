# ESP32-S3 Super Mini 串口多路复用控制器

基于 ESP32-S3 Super Mini 开发板的固件，实现 8 路串口多路复用切换，并将 USB 串口透传到 EMS 设备 UART 接口。

## 功能

- **8 路通道切换**：通过 GPIO11/12/13 三根引脚的高低电平组合（3 位二进制），控制外部 8 路多路复用器（如 74HC4051）选择当前激活通道
- **USB 串口透传**：未识别为控制命令的串口输入，会被原样转发到 EMS 设备（UART2，GPIO17=TX / GPIO18=RX）
- **EMS 数据回传**：EMS UART 收到的数据实时回传到 USB 串口，方便调试和监控
- **板载 RGB LED**：GPIO48 连接 FastLED 可寻址 LED（当前未启用控制逻辑，留作扩展）

## 硬件

| 用途 | 引脚 |
|------|------|
| 多路复用器 A 位 | GPIO11 |
| 多路复用器 B 位 | GPIO12 |
| 多路复用器 C 位 | GPIO13 |
| EMS UART TX | GPIO17 |
| EMS UART RX | GPIO18 |
| 板载 RGB LED | GPIO48 |

> 注意：GPIO11/12 在部分 ESP32-S3 变体上与 SPI Flash 复用，使用前请确认你的板子引脚实际可用。详见 [AVAILABLE_PINS.md](AVAILABLE_PINS.md)。

## 串口命令

波特率：**115200**

| 命令 | 说明 |
|------|------|
| `0` ~ `7` | 直接输入单个数字切换到对应通道 |
| `select 0` ~ `select 7` | 用 `select N` 格式切换通道 |

切换成功后串口会回复 `selected N`。

其他非命令输入会被透传到 EMS UART。

## 构建与烧录

项目使用 [PlatformIO](https://platformio.org/) 构建，目标板为 ESP32-S3 Super Mini。

```shell
# 安装 PlatformIO CLI（如尚未安装）
pip install platformio

# 编译
pio run

# 编译并烧录
pio run --target upload

# 打开串口监视器
pio device monitor
```

## 依赖库

- [FastLED](https://github.com/FastLED/FastLED)
- [EspSoftwareSerial](https://github.com/plerup/espsoftwareserial)
