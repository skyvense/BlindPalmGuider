#include <HardwareSerial.h>
#include <FastLED.h>
#include <SoftwareSerial.h>

#define LED_PIN 48
#define NUM_LEDS 1
#define CHANNEL_COUNT 8

#define MUX_A 11
#define MUX_B 12
#define MUX_C 13

#define SWITCH_WAIT_MS   0
#define SEND_INTERVAL_MS 10

CRGB leds[NUM_LEDS];
HardwareSerial EMS_UART(2);         // UART2: RX=GPIO17, TX=GPIO18
SoftwareSerial Camera_UART(8, 9);   // RX=8, TX=9

char serialLineBuffer[32] = {0};
uint8_t serialLineLength = 0;

// per-channel running state
bool chRunning[CHANNEL_COUNT] = {false};

// 3x3 grid → channel mapping (center [1][1] skipped)
// ch: 0=TL 1=T 2=TR 3=L 4=R 5=BL 6=B 7=BR
static const uint8_t CH_ROW[CHANNEL_COUNT] = {0, 0, 0, 1, 1, 2, 2, 2};
static const uint8_t CH_COL[CHANNEL_COUNT] = {0, 1, 2, 0, 2, 0, 1, 2};

// block boundaries for 25x25 → 3x3 average
static const uint8_t BLK_START[3] = {0,  8, 17};
static const uint8_t BLK_END[3]   = {8, 17, 25};

// ── mux ──────────────────────────────────────────────────────────────────────

void selectChannel(uint8_t ch) {
  digitalWrite(MUX_A, (ch >> 0) & 1 ? HIGH : LOW);
  digitalWrite(MUX_B, (ch >> 1) & 1 ? HIGH : LOW);
  digitalWrite(MUX_C, (ch >> 2) & 1 ? HIGH : LOW);
}

void switchChannel(uint8_t ch) {
  selectChannel(ch);
  if (SWITCH_WAIT_MS > 0) delay(SWITCH_WAIT_MS);
}

// ── EMS ──────────────────────────────────────────────────────────────────────

void sendEMSCmd(const char* cmd, uint8_t ch) {
  Serial.print("[ch"); Serial.print(ch); Serial.print("] >> "); Serial.println(cmd);
  // EMS_UART.print(cmd);
  // EMS_UART.print("\n");
  // uint32_t t0 = millis();
  // bool printed = false;
  // while (millis() - t0 < SEND_INTERVAL_MS) {
  //   if (EMS_UART.available()) {
  //     if (!printed) {
  //       Serial.print("[ch"); Serial.print(ch); Serial.print("] ");
  //       printed = true;
  //     }
  //     Serial.write(EMS_UART.read());
  //   }
  // }
  // if (printed) Serial.println();
}

void initAllChannels() {
  Serial.println("init channels");
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    switchChannel(ch);
    sendEMSCmd("FE 00 00 01 100", ch);
    sendEMSCmd("start_ems", ch);
    chRunning[ch] = true;
  }
}

// dist_m → EMS value 1-5, or 0 if >1m
int distToVal(float dist_m) {
  if (dist_m > 1.0f) return 0;
  int v = (int)round(5.0f - dist_m / 1.0f * 4.0f);
  if (v < 1) v = 1;
  if (v > 5) v = 5;
  return v;
}

void updateEMSChannel(uint8_t ch, float dist_m) {
  switchChannel(ch);
  int val = distToVal(dist_m);

  if (val == 0) {
    if (chRunning[ch]) {
      sendEMSCmd("stop_ems", ch);
      chRunning[ch] = false;
    }
  } else {
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "FE 00 00 %d 100", val);
    sendEMSCmd(cmd, ch);
    if (!chRunning[ch]) {
      sendEMSCmd("start_ems", ch);
      chRunning[ch] = true;
    }
  }
}

// ── Camera ───────────────────────────────────────────────────────────────────

// pixel value → distance in meters (UNIT=0: dist_mm = (p/5.1)^2)
float pixelToMeters(uint8_t p) {
  float d = p / 5.1f;
  return (d * d) / 1000.0f;
}

// average downscale 25x25 → 3x3
// insertion sort on small array
static void sortBytes(uint8_t* arr, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    uint8_t key = arr[i], j = i;
    while (j > 0 && arr[j-1] > key) { arr[j] = arr[j-1]; j--; }
    arr[j] = key;
  }
}

void computeGrid(const uint8_t* buf, float grid[3][3]) {
  for (uint8_t r = 0; r < 3; r++) {
    for (uint8_t c = 0; c < 3; c++) {
      // collect block pixels (max 9x9 = 81)
      uint8_t pixels[81];
      uint8_t cnt = 0;
      for (uint8_t row = BLK_START[r]; row < BLK_END[r]; row++)
        for (uint8_t col = BLK_START[c]; col < BLK_END[c]; col++)
          pixels[cnt++] = buf[row * 25 + col];

      sortBytes(pixels, cnt);

      // trim 10% from each end
      uint8_t trim = cnt / 10;
      uint32_t sum = 0;
      uint8_t used = cnt - 2 * trim;
      for (uint8_t i = trim; i < cnt - trim; i++) sum += pixels[i];
      grid[r][c] = (float)sum / used;
    }
  }
}

// read one frame; returns true on success
bool readCameraFrame(uint8_t* buf) {
  uint32_t deadline = millis() + 500;

  // sync: find 0x00 0xFF
  uint8_t prev = 0xFF;
  bool synced = false;
  while (millis() < deadline) {
    if (!Camera_UART.available()) continue;
    uint8_t b = Camera_UART.read();
    if (prev == 0x00 && b == 0xFF) { synced = true; break; }
    prev = b;
  }
  if (!synced) return false;

  // skip 2-byte length + 16-byte metadata = 18 bytes
  for (uint8_t i = 0; i < 18; ) {
    if (millis() >= deadline) return false;
    if (Camera_UART.available()) { Camera_UART.read(); i++; }
  }

  // read 625 pixels
  for (uint16_t i = 0; i < 625; ) {
    if (millis() >= deadline) return false;
    if (Camera_UART.available()) { buf[i++] = Camera_UART.read(); }
  }

  // skip checksum + tail
  for (uint8_t i = 0; i < 2; ) {
    if (millis() >= deadline) return false;
    if (Camera_UART.available()) { Camera_UART.read(); i++; }
  }

  return true;
}

// ── manual commands ───────────────────────────────────────────────────────────

void pollEMS() {
  for (int i = 0; i < 20; i++) {
    EMS_UART.print("FE 00 00 01 30\n");
    uint32_t start = millis();
    bool printed = false;
    while (millis() - start < 500) {
      if (EMS_UART.available() > 0) {
        if (!printed) {
          Serial.print("["); Serial.print(i + 1); Serial.print("] ");
          printed = true;
        }
        Serial.write(EMS_UART.read());
      }
    }
  }
}

void runAutoTest(uint32_t switchWait, uint32_t sendInterval) {
  Serial.print("\n=== autotest switchWait=");
  Serial.print(switchWait);
  Serial.print("ms sendInterval=");
  Serial.print(sendInterval);
  Serial.println("ms ===");

  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    Serial.print("\n--- ch"); Serial.print(ch); Serial.println(" ---");

    uint32_t t0 = millis();
    switchChannel(ch);
    Serial.print("switch: "); Serial.print(millis() - t0); Serial.println("ms");

    uint32_t ws = millis();
    delay(switchWait);
    Serial.print("switchWait: "); Serial.print(millis() - ws); Serial.println("ms");

    for (int i = 0; i < 10; i++) {
      uint32_t sendStart = millis();
      EMS_UART.print("FE 00 00 01 30\n");

      bool got = false, printed = false;
      while (millis() - sendStart < sendInterval) {
        if (EMS_UART.available() > 0) {
          if (!printed) {
            Serial.print("["); Serial.print(i + 1); Serial.print("] +");
            Serial.print(millis() - sendStart); Serial.print("ms: ");
            printed = true; got = true;
          }
          Serial.write(EMS_UART.read());
        }
      }
      Serial.print("  interval: "); Serial.print(millis() - sendStart); Serial.println("ms");
      if (!got) {
        Serial.print("["); Serial.print(i + 1); Serial.println("] no response -> stop");
        return;
      }
    }
  }
  Serial.println("\n=== autotest done ===");
}

bool handleSerialCommand() {
  // single digit channel select
  if (serialLineLength == 1 &&
      serialLineBuffer[0] >= '0' &&
      serialLineBuffer[0] < ('0' + CHANNEL_COUNT)) {
    uint8_t ch = serialLineBuffer[0] - '0';
    switchChannel(ch);
    Serial.print("selected "); Serial.println(ch);
    pollEMS();
    return true;
  }

  // "select N"
  if (serialLineLength == 8 &&
      serialLineBuffer[0] == 's' && serialLineBuffer[1] == 'e' &&
      serialLineBuffer[2] == 'l' && serialLineBuffer[3] == 'e' &&
      serialLineBuffer[4] == 'c' && serialLineBuffer[5] == 't' &&
      serialLineBuffer[6] == ' ' &&
      serialLineBuffer[7] >= '0' && serialLineBuffer[7] < ('0' + CHANNEL_COUNT)) {
    uint8_t ch = serialLineBuffer[7] - '0';
    switchChannel(ch);
    Serial.print("selected "); Serial.println(ch);
    pollEMS();
    return true;
  }

  // "autotest [switchWait [sendInterval]]"
  if (serialLineLength >= 8 &&
      serialLineBuffer[0] == 'a' && serialLineBuffer[1] == 'u' &&
      serialLineBuffer[2] == 't' && serialLineBuffer[3] == 'o' &&
      serialLineBuffer[4] == 't' && serialLineBuffer[5] == 'e' &&
      serialLineBuffer[6] == 's' && serialLineBuffer[7] == 't') {
    uint32_t sw = 1000, si = 500;
    if (serialLineLength > 9) sw = atoi(serialLineBuffer + 9);
    uint8_t sp = 0;
    for (uint8_t k = 9; k < serialLineLength; k++) {
      if (serialLineBuffer[k] == ' ') { sp = k; break; }
    }
    if (sp > 0 && sp + 1 < serialLineLength) si = atoi(serialLineBuffer + sp + 1);
    runAutoTest(sw, si);
    return true;
  }

  return false;
}

void flushBufferedSerialToEMS() {
  for (uint8_t i = 0; i < serialLineLength; ++i)
    EMS_UART.write(serialLineBuffer[i]);
}

// ── setup / loop ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  EMS_UART.begin(115200, SERIAL_8N1, 17, 18);
  Camera_UART.begin(115200);

  FastLED.addLeds<SK6812, LED_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Blue;
  FastLED.show();

  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_C, OUTPUT);
  selectChannel(0);

  // configure camera
  delay(500);
  Camera_UART.print("AT+FPS=10\r");
  delay(200);
  while (Camera_UART.available()) Camera_UART.read();
  Camera_UART.print("AT+BINN=4\r");
  delay(200);
  while (Camera_UART.available()) Camera_UART.read();
  Camera_UART.print("AT+DISP=5\r");
  delay(2000);  // wait for camera to start streaming
  while (Camera_UART.available()) Camera_UART.read();
  Serial.println("camera configured");

  initAllChannels();

  leds[0] = CRGB::Green;
  FastLED.show();
  Serial.println("ready");
}

void loop() {
  // handle USB serial commands (non-blocking check)
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\r' || ch == '\n') {
      if (serialLineLength > 0) {
        if (!handleSerialCommand()) {
          flushBufferedSerialToEMS();
          EMS_UART.write(ch);
        }
        serialLineLength = 0;
      }
      continue;
    }
    if (serialLineLength < sizeof(serialLineBuffer) - 1) {
      serialLineBuffer[serialLineLength++] = ch;
      serialLineBuffer[serialLineLength] = '\0';
    } else {
      flushBufferedSerialToEMS();
      EMS_UART.write(ch);
      serialLineLength = 0;
    }
  }

  // read one ToF frame
  static uint8_t frameBuf[625];
  if (!readCameraFrame(frameBuf)) {
    Serial.println("frame timeout");
    leds[0] = CRGB::Red;
    FastLED.show();
    return;
  }

  leds[0] = CRGB::Green;
  FastLED.show();

  // downscale to 3x3
  float grid[3][3];
  computeGrid(frameBuf, grid);

  // print 3x3 grid (distance in meters)
  Serial.println("grid (m):");
  for (uint8_t r = 0; r < 3; r++) {
    for (uint8_t c = 0; c < 3; c++) {
      float dist_m = pixelToMeters((uint8_t)grid[r][c]);
      if (r == 1 && c == 1) {
        Serial.print("  -- ");
      } else {
        char buf[8];
        dtostrf(dist_m, 4, 2, buf);
        Serial.print(buf);
        Serial.print(" ");
      }
    }
    Serial.println();
  }

  // update all 8 EMS channels
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    float dist_m = pixelToMeters((uint8_t)grid[CH_ROW[ch]][CH_COL[ch]]);
    updateEMSChannel(ch, dist_m);
  }
}
