# BlindPalmGuider

ESP32-S3 Super Mini firmware for a palm-worn navigation aid for the visually impaired. Reads depth from a MaixSense-A010 ToF camera, maps the scene into 8 directional zones, and drives 8 independent EMS (Electrical Muscle Stimulation) devices through a serial multiplexer. Connects to WiFi and serves a real-time web dashboard for monitoring and control.

---

## How It Works

```
[ToF Camera] ──HardwareSerial(1)──► [ESP32-S3] ──HardwareSerial──► [8-ch Mux] ──► [EMS ×8]
                                      │
                                   WiFi AP
                                      │
                                  [Browser]
```

1. The camera streams 25×25 depth frames at 10 fps over UART.
2. The firmware downscales each frame to a 3×3 grid (block averaging, 10% outlier trim).
3. The 8 surrounding cells (center discarded) map to 8 EMS channels — TL, T, TR, L, R, BL, B, BR.
4. For each channel the mux is switched, an EMS intensity command is sent, and the device's response is read — all within a single frame cycle.
5. The web dashboard reflects live distance data and accepts control input with ~300 ms latency.

---

## Hardware

### Components

| Component | Description |
|-----------|-------------|
| ESP32-S3 Super Mini | Main controller (ESP32S3FH4R2) |
| MaixSense-A010 | ToF depth camera, 25×25 px, up to 2 Mbps UART |
| 8-channel serial mux | Hardware UART multiplexer with A/B/C address pins |
| EMS devices ×8 | Connected to mux outputs TX0–TX7 / RX0–RX7 |

### Wiring

| Signal | ESP32-S3 GPIO |
|--------|--------------|
| Camera RX (cam TX → ESP RX) | GPIO 8 |
| Camera TX (cam RX → ESP TX) | GPIO 9 |
| EMS UART RX | GPIO 17 |
| EMS UART TX | GPIO 18 |
| Mux address A (bit 0) | GPIO 11 |
| Mux address B (bit 1) | GPIO 12 |
| Mux address C (bit 2) | GPIO 13 |
| On-board RGB LED | GPIO 48 |

### Channel Mapping (3×3 → 8 EMS)

```
 ch0 (TL) │ ch1 (T)  │ ch2 (TR)
──────────┼──────────┼──────────
 ch3 (L)  │  center  │ ch4 (R)
           │ (unused) │
──────────┼──────────┼──────────
 ch5 (BL) │ ch6 (B)  │ ch7 (BR)
```

The center block is measured but not assigned to a channel. If the center distance is closer than a surrounding cell, that cell inherits the center value (occlusion propagation).

---

## Firmware

### Key Parameters (`src/main.cpp`)

| Define | Default | Description |
|--------|---------|-------------|
| `SWITCH_WAIT_MS` | `0` | Delay after mux switch before sending |
| `SEND_INTERVAL_MS` | `10` | Time to wait for EMS response per command |
| `WIFI_SSID` | `"your-ssid"` | WiFi station SSID to connect to |
| `WIFI_PASS` | `"your-pass"` | WiFi station password |

### EMS State Machine (per channel)

```
Startup   → all channels disabled (no commands sent)

Enable    → FE 00 00 01 100  (set intensity 1)
          → start_ems

Each frame, per channel:
  dist > threshold  →  if running: stop_ems  →  state = stopped
  dist ≤ threshold  →  FE 00 00 {1–30} 100   (intensity mapped from distance)
                        if stopped: start_ems  →  state = running
```

Intensity mapping: `0 m → chMax`, `threshold m → chMin` (linear). Default range 1–5, adjustable per channel up to 30.

### Camera Setup (AT commands on boot)

```
AT+FPS=10    10 frames per second
AT+BINN=4    25×25 resolution (4×4 binning)
AT+DISP=5    LCD + UART output
```

Camera ISP can be toggled at runtime via the web UI (`AT+ISP=0` / `AT+ISP=1`).

### Frame Format (BINN=4)

```
0x00 0xFF          2-byte header
[LEN_H] [LEN_L]    2-byte payload length
[16 bytes]         metadata (discarded)
[625 bytes]        25×25 pixel depth data, row-major, uint8
[CKSUM]            checksum (discarded)
0xDD               tail byte
```

Pixel → distance: `dist_mm = (pixel / 5.1)²` (UNIT=0 default)

---

## Web Dashboard

Connect the device to your WiFi (configure `WIFI_SSID` / `WIFI_PASS` in `src/main.cpp`) and open the device's IP in a browser.

### Sections

**Depth Heatmap** — Live 3×3 color grid. Red = near, blue = far/off. Heatmap always reflects raw depth regardless of whether EMS channels are enabled. Rotation buttons (↺ ↻) are in progress.

**Channels** — Same 3×3 layout with per-channel toggle switches, running/stopped status, and the last EMS command sent.

**Distance Threshold** — Global slider (0.1 m – 3.0 m). Cells beyond this distance trigger `stop_ems`.

**Intensity Range** — Per-channel dual-handle slider setting the min and max intensity values (1–30) mapped to the distance range.

**All Channels** — Master toggle. Turns all 8 channels on (sends init + `start_ems`) or off (`stop_ems`) simultaneously.

**Camera** — Toggles `AT+ISP=1` / `AT+ISP=0` to start or stop the ToF sensor.

---

## Serial Commands (USB, 115200 baud)

| Command | Description |
|---------|-------------|
| `0` – `7` | Manually switch mux to channel N and run 20× poll |
| `select N` | Same as above (verbose form) |
| `autotest` | Run timing test with default params (switchWait=1000ms, sendInterval=500ms) |
| `autotest W I` | Run timing test with switchWait=W ms, sendInterval=I ms |

Any unrecognised input is forwarded verbatim to the EMS UART.

---

## Build & Flash

```bash
# Install PlatformIO if needed
pip install platformio

# Build
pio run

# Build and flash (auto-detect port)
pio run --target upload

# Open serial monitor
pio device monitor
```

Target board: `dfrobot_firebeetle2_esp32s3` (pin-compatible with ESP32-S3 Super Mini).

### Dependencies

| Library | Source |
|---------|--------|
| FastLED | `fastled/FastLED` |
| WiFi, WebServer | ESP32 Arduino core (built-in) |

---

## Files

```
src/main.cpp        Firmware (all logic in one file)
doc/plan.md         Architecture notes and design decisions
visualize.py        Python serial heatmap visualizer (matplotlib)
platformio.ini      Build configuration
AVAILABLE_PINS.md   GPIO reference for ESP32-S3 Super Mini
```

### Python Visualizer

Reads `grid (m):` blocks from the USB serial port and renders a live heatmap.

```bash
pip install pyserial matplotlib numpy -i https://pypi.tuna.tsinghua.edu.cn/simple
python3 visualize.py
```

Edit `PORT` at the top of `visualize.py` if your device is not on `/dev/cu.usbmodem11201`.
