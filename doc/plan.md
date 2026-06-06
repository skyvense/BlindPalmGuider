# 实现计划：ToF相机 + 8路EMS通道自动控制

## 目标

用一个 MaixSense-A010 ToF相机采集场景深度，将25×25深度帧**平均缩放**到3×3，去掉中心点后得到8个方向的距离值，每帧同步更新8路EMS设备的开关状态和强度。

---

## 硬件连接

| 设备 | 接口 | 引脚 |
|------|------|------|
| ToF 相机 (MaixSense-A010) | SoftwareSerial | RX=GPIO8, TX=GPIO9 |
| EMS 设备（共用总线，经多路复用） | HardwareSerial UART2 | RX=GPIO17, TX=GPIO18 |
| 多路复用器控制 A/B/C | GPIO | 11 / 12 / 13 |

---

## 关键参数 (define)

```cpp
#define SWITCH_WAIT_MS   0    // 切换通道后等待时间
#define SEND_INTERVAL_MS 10   // 发送EMS命令后等待响应时间
```

---

## ToF 相机配置（setup）

AT 指令以 `\r` 结尾，每条间隔 150ms：

```
AT+FPS=10\r     → 10帧/秒
AT+BINN=4\r     → 25×25 分辨率
AT+DISP=4\r     → 开启UART输出
```

---

## 数据帧格式 (BINN=4, 共647字节)

```
[0x00][0xFF]    包头（滑动窗口同步）
[LEN_H][LEN_L]  2字节长度（跳过）
[...16字节...]  元数据（跳过）
[625字节]       像素，25×25，uint8，行优先
[CKSUM]         1字节（跳过）
[0xDD]          包尾（跳过）
```

---

## 25×25 → 3×3 平均缩放

```
块边界（行/列相同）：[0,8), [8,17), [17,25)

grid[r][c] = mean(buf[row*25+col])，row∈row_range[r]，col∈col_range[c]
```

---

## 通道映射（舍弃中心 grid[1][1]）

```
ch0=grid[0][0]  ch1=grid[0][1]  ch2=grid[0][2]
ch3=grid[1][0]      (舍弃)      ch4=grid[1][2]
ch5=grid[2][0]  ch6=grid[2][1]  ch7=grid[2][2]

方向：0左上 1上 2右上 3左 4右 5左下 6下 7右下
```

---

## EMS 状态机（每通道独立）

```
启动时：
  → 发 "FE 00 00 01 100\n" + "start_ems\n"
  → 标记 state = RUNNING

每帧：
  dist_m = (pixel/5.1)² / 1000.0

  if dist_m > 2.0:
    if state == RUNNING:
      → 发 "stop_ems\n"
      → state = STOPPED

  else:
    val = clamp(round(5 - dist_m/2.0*4), 1, 5)   // 0m→5, 2m→1
    → 发 "FE 00 00 {val} 100\n"
    if state == STOPPED:
      → 发 "start_ems\n"
      → state = RUNNING
```

---

## 主循环流程

```
loop:
  1. flush Camera_UART（丢弃积压旧帧）
  2. 读一帧（超时500ms）
       - 滑动匹配 0x00 0xFF
       - 跳过18字节（2+16）
       - 读625字节到 buf[]
       - 跳过2字节
  3. computeGrid(buf) → grid[3][3]
  4. for ch = 0..7:
       switchChannel(ch)
       delay(SWITCH_WAIT_MS)
       更新EMS状态机
       发指令 + 等 SEND_INTERVAL_MS + 打印响应
  5. goto 1
```

---

## 帧率分析

- 相机 10fps → 100ms/帧
- 8路 × 10ms = 80ms/帧用于EMS通信
- 625字节@115200 ≈ 55ms，需在帧间隙完成读取
- 若丢帧严重可降到 AT+FPS=5
