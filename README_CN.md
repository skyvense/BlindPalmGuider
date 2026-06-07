# BlindPalmGuider

基于 ESP32-S3 Super Mini 的手掌佩戴式视障导航辅助设备固件。通过 MaixSense-A010 ToF 摄像头读取深度数据，将场景划分为 8 个方向区域，并通过串口多路复用器驱动 8 路独立的 EMS（电肌肉刺激）设备。连接 WiFi 后，提供实时网页控制面板用于监控与调节。

---

## 工作原理

```
[ToF 摄像头] ──HardwareSerial(1)──► [ESP32-S3] ──HardwareSerial──► [8路多路复用器] ──► [EMS ×8]
                                         │
                                       WiFi
                                         │
                                      [浏览器]
```

1. 摄像头通过 UART 以 10 fps 持续输出 25×25 深度帧。
2. 固件将每帧降采样为 3×3 网格（分块平均，去除 10% 异常值）。
3. 外围 8 个格子（中心丢弃）映射到 8 路 EMS 通道——左上、上、右上、左、右、左下、下、右下。
4. 每个通道依次切换多路复用器、发送 EMS 强度指令并读取响应——全部在一个帧周期内完成。
5. 网页控制面板实时反映距离数据，控制操作延迟约 300 ms。

---

## 硬件

### 元器件清单

| 元器件 | 说明 |
|--------|------|
| ESP32-S3 Super Mini | 主控（ESP32S3FH4R2） |
| MaixSense-A010 | ToF 深度摄像头，25×25 像素，UART 最高 2 Mbps |
| 8 路串口多路复用器 | 硬件 UART 多路复用器，A/B/C 地址引脚 |
| EMS 设备 ×8 | 接至多路复用器输出 TX0–TX7 / RX0–RX7 |

### 接线

| 信号 | ESP32-S3 GPIO |
|------|--------------|
| 摄像头 RX（摄像头 TX → ESP RX） | GPIO 8 |
| 摄像头 TX（摄像头 RX → ESP TX） | GPIO 9 |
| EMS UART RX | GPIO 17 |
| EMS UART TX | GPIO 18 |
| 多路复用器地址 A（bit 0） | GPIO 11 |
| 多路复用器地址 B（bit 1） | GPIO 12 |
| 多路复用器地址 C（bit 2） | GPIO 13 |
| 板载 RGB LED | GPIO 48 |

### 通道映射（3×3 → 8 路 EMS）

```
 ch0（左上） │ ch1（上）  │ ch2（右上）
────────────┼────────────┼────────────
 ch3（左）  │   中心     │ ch4（右）
            │  （未用）  │
────────────┼────────────┼────────────
 ch5（左下）│ ch6（下）  │ ch7（右下）
```

中心格只做测量，不分配通道。若中心距离近于某相邻格，该格继承中心值（遮挡传播）。

---

## 固件

### 关键参数（`src/main.cpp`）

| 宏定义 | 默认值 | 说明 |
|--------|--------|------|
| `SWITCH_WAIT_MS` | `0` | 多路复用器切换后发送前的等待时间 |
| `SEND_INTERVAL_MS` | `10` | 每条 EMS 指令等待响应的时间 |
| `WIFI_SSID` | `"your-ssid"` | 要连接的 WiFi 名称 |
| `WIFI_PASS` | `"your-pass"` | WiFi 密码 |

### EMS 状态机（每通道）

```
启动       → 所有通道禁用（不发送指令）

启用       → FE 00 00 01 100（设置强度为 1）
           → start_ems

每帧，每通道：
  距离 > 阈值  →  若运行中：stop_ems  →  状态 = 停止
  距离 ≤ 阈值  →  FE 00 00 {1–30} 100（强度按距离线性映射）
                   若已停止：start_ems  →  状态 = 运行
```

强度映射：`0 m → chMax`，`阈值距离 → chMin`（线性）。默认范围 1–5，每通道可单独调节，最大 30。

### 摄像头初始化（开机 AT 指令）

```
AT+FPS=10    帧率 10 fps
AT+BINN=4    分辨率 25×25（4×4 binning）
AT+DISP=5    LCD + UART 同时输出
```

摄像头 ISP 可在运行时通过网页控制面板切换（`AT+ISP=0` / `AT+ISP=1`）。

### 帧格式（BINN=4）

```
0x00 0xFF          2 字节帧头
[LEN_H] [LEN_L]    2 字节负载长度
[16 字节]          元数据（丢弃）
[625 字节]         25×25 像素深度数据，行优先，uint8
[CKSUM]            校验和（丢弃）
0xDD               帧尾
```

像素转距离：`dist_mm = (pixel / 5.1)²`（默认 UNIT=0）

---

## 网页控制面板

在 `src/main.cpp` 中配置 `WIFI_SSID` / `WIFI_PASS`，设备连接 WiFi 后，在浏览器中打开设备 IP 即可访问。

### 功能区

**深度热力图** — 实时 3×3 彩色网格。红色 = 近，蓝色 = 远/关闭。热力图始终显示原始深度，与 EMS 通道是否开启无关。旋转按钮（↺ ↻）功能开发中。

**通道** — 同样的 3×3 布局，每通道独立开关、运行/停止状态及最近一次 EMS 指令。

**距离阈值** — 全局滑块（0.1 m – 3.0 m）。超出该距离的格子触发 `stop_ems`。

**强度范围** — 每通道双滑块，设置强度最小值和最大值（1–30），与距离范围线性映射。

**全部通道** — 主开关。一键开启所有 8 路（发送初始化 + `start_ems`）或关闭（`stop_ems`）。

**摄像头** — 切换 `AT+ISP=1` / `AT+ISP=0`，启动或停止 ToF 传感器。

---

## 串口指令（USB，波特率 115200）

| 指令 | 说明 |
|------|------|
| `0` – `7` | 手动切换多路复用器至通道 N，并执行 20 次轮询 |
| `select N` | 同上（详细写法） |
| `autotest` | 使用默认参数运行时序测试（switchWait=1000ms，sendInterval=500ms） |
| `autotest W I` | 使用 switchWait=W ms、sendInterval=I ms 运行时序测试 |

无法识别的输入将原样转发至 EMS UART。

---

## 编译与烧录

```bash
# 安装 PlatformIO（如未安装）
pip install platformio

# 编译
pio run

# 编译并烧录（自动识别端口）
pio run --target upload

# 打开串口监视器
pio device monitor
```

目标开发板：`dfrobot_firebeetle2_esp32s3`（引脚兼容 ESP32-S3 Super Mini）。

### 依赖库

| 库 | 来源 |
|----|------|
| FastLED | `fastled/FastLED` |
| WiFi, WebServer | ESP32 Arduino core（内置） |

---

## 文件结构

```
src/main.cpp        固件（全部逻辑集中在单文件）
doc/plan.md         架构说明与设计决策
visualize.py        Python 串口热力图可视化工具（matplotlib）
platformio.ini      编译配置
AVAILABLE_PINS.md   ESP32-S3 Super Mini GPIO 参考
```

### Python 可视化工具

从 USB 串口读取 `grid (m):` 数据块并渲染实时热力图。

```bash
pip install pyserial matplotlib numpy -i https://pypi.tuna.tsinghua.edu.cn/simple
python3 visualize.py
```

如果设备不在 `/dev/cu.usbmodem11201`，请修改 `visualize.py` 顶部的 `PORT` 变量。
