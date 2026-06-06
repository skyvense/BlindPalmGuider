# ESP32-S3 Super Mini UART 可用引脚

## ❌ 不可用引脚
- **GPIO11, GPIO12** - SPI Flash 专用，不可用！
- **GPIO8, GPIO9** - USB-JTAG/下载用，不建议用

## ✅ 推荐可用引脚

### 方案1: GPIO6/7 (推荐)
```cpp
#define CAM_RX_PIN 6   // 相机 TX -> ESP32 RX
#define CAM_TX_PIN 7   // 相机 RX -> ESP32 TX
```

### 方案2: GPIO4/5
```cpp
#define CAM_RX_PIN 4   // 相机 TX -> ESP32 RX
#define CAM_TX_PIN 5   // 相机 RX -> ESP32 TX
```

### 方案3: GPIO15/16
```cpp
#define CAM_RX_PIN 15  // 相机 TX -> ESP32 RX
#define CAM_TX_PIN 16  // 相机 RX -> ESP32 TX
```

### 方案4: GPIO17/18
```cpp
#define CAM_RX_PIN 17  // 相机 TX -> ESP32 RX
#define CAM_TX_PIN 18  // 相机 RX -> ESP32 TX
```

## Super Mini 引脚位置图

```
        ┌─────────────────┐
   3V3  │ ●           ● │  GND
   GPIO0│ ●           ● │  GPIO6
   GPIO1│ ●           ● │  GPIO7  ← 推荐
   GPIO2│ ●           ● │  GPIO8  (USB)
   GPIO3│ ●           ● │  GPIO9  (USB)
   GPIO4│ ●           ● │  GPIO10
   GPIO5│ ●           ● │  GPIO11 (不可用)
   GPIO6│ ●           ● │  GPIO12 (不可用)
   GPIO7│ ●           ● │  GPIO13
   GPIO8│ ●           ● │  GPIO14
   GPIO9│ ●           ● │  GPIO15
   GPIO10│●           ● │  GPIO16
   GPIO11│●           ● │  GPIO17
   GPIO12│●           ● │  GPIO18
        └─────────────────┘
```

## 当前代码引脚
查看 main.cpp 中的定义，修改为你实际接线的引脚。
