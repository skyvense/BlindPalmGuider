#include <HardwareSerial.h>
#include <FastLED.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>

#define LED_PIN 48
#define NUM_LEDS 1
#define CHANNEL_COUNT 8

#define MUX_A 11
#define MUX_B 12
#define MUX_C 13

#define SWITCH_WAIT_MS   0
#define SEND_INTERVAL_MS 10

#define AP_SSID "EMS-Control"
#define AP_PASS "12345678"

CRGB leds[NUM_LEDS];
HardwareSerial EMS_UART(2);
SoftwareSerial Camera_UART(8, 9);
WebServer server(80);

char serialLineBuffer[32] = {0};
uint8_t serialLineLength = 0;

// per-channel state
bool chRunning[CHANNEL_COUNT] = {false};
bool chEnabled[CHANNEL_COUNT] = {false};  // all off by default

// forward declarations
void selectChannel(uint8_t ch);
void switchChannel(uint8_t ch);
void sendEMSCmd(const char* cmd, uint8_t ch);
float lastDist[CHANNEL_COUNT] = {9.9f};
char  lastCmd[CHANNEL_COUNT][32] = {""};
float lastGrid[3][3] = {{}};
bool  cameraEnabled = true;

// 3x3 grid → channel (skip center [1][1])
// ch: 0=TL 1=T 2=TR 3=L 4=R 5=BL 6=B 7=BR
static const uint8_t CH_ROW[CHANNEL_COUNT] = {0, 0, 0, 1, 1, 2, 2, 2};
static const uint8_t CH_COL[CHANNEL_COUNT] = {0, 1, 2, 0, 2, 0, 1, 2};

static const uint8_t BLK_START[3] = {0,  8, 17};
static const uint8_t BLK_END[3]   = {8, 17, 25};

// ── Web page ──────────────────────────────────────────────────────────────────

static const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>EMS Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;font-family:sans-serif}
body{background:#1a1a2e;color:#eee;padding:16px;max-width:480px;margin:0 auto}
h2{font-size:13px;opacity:.5;text-transform:uppercase;letter-spacing:1px;margin-bottom:8px}
.g3{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-bottom:20px}
/* heatmap cells */
.hcell{border-radius:8px;padding:12px 4px;text-align:center;transition:background .4s;min-height:72px;display:flex;flex-direction:column;justify-content:center}
.hcell .hn{font-size:10px;opacity:.6;margin-bottom:2px}
.hcell .hd{font-size:20px;font-weight:700}
.hcell.hcenter{background:#232336!important}
/* channel grid cells */
.ccell{border-radius:8px;padding:8px 6px;background:#16213e;min-height:90px;display:flex;flex-direction:column;justify-content:space-between}
.ccell.cdisabled{opacity:.45}
.ccell.ccenter{background:#1a1a2e!important;border:1px dashed #333}
.cn{font-size:11px;font-weight:700;margin-bottom:2px}
.cs{font-size:9px;opacity:.6;margin-bottom:4px}
.ccmd{font-size:8px;opacity:.7;word-break:break-all;min-height:20px;flex:1;margin-bottom:4px}
/* toggle */
.toggle{position:relative;width:44px;height:24px;flex-shrink:0;align-self:flex-end}
.toggle input{opacity:0;width:0;height:0}
.slider{position:absolute;cursor:pointer;inset:0;background:#444;border-radius:24px;transition:.3s}
.slider:before{position:absolute;content:"";height:18px;width:18px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:.3s}
input:checked+.slider{background:#43a047}
input:checked+.slider:before{transform:translateX(20px)}
/* bottom controls */
.row{display:flex;justify-content:space-between;align-items:center;
     padding:10px 14px;background:#16213e;border-radius:8px;margin-bottom:8px}
.row b{font-size:14px}
.row small{font-size:11px;opacity:.55;display:block;margin-top:2px}
.btn{width:100%;padding:12px;border:none;color:#fff;border-radius:8px;
     font-size:14px;cursor:pointer;margin-bottom:8px;transition:.3s}
.bon{background:#0d47a1}.boff{background:#555}
.master{border:1px solid #43a047}
</style>
</head>
<body>

<h2>Depth Heatmap</h2>
<div class="g3" id="hmap"></div>

<h2>Channels</h2>
<div class="g3" id="cgrid"></div>

<div class="row master">
  <div><b>All Channels</b><small id="masterSub">all off</small></div>
  <label class="toggle"><input type="checkbox" id="masterTog" onchange="toggleAll()"><span class="slider"></span></label>
</div>
<button class="btn bon" id="camBtn" onclick="toggleCamera()">Camera: ON</button>

<script>
const CH_NAME=['TL','T','TR','L','R','BL','B','BR'];
const CH_POS=[[0,0],[0,1],[0,2],[1,0],[1,2],[2,0],[2,1],[2,2]];

function distColor(dist,enabled){
  if(!enabled||dist>=1)return'#1a2a4a';
  const t=1-dist;
  return`rgb(${Math.round(t*230)},${Math.round((0.5-Math.abs(t-0.5))*200)},${Math.round((1-t)*160)})`;
}

// ── build heatmap (pure distance view) ──
const hmap=document.getElementById('hmap');
const hcells=[];
for(let r=0;r<3;r++)for(let c=0;c<3;c++){
  const el=document.createElement('div');
  const center=r===1&&c===1;
  el.className='hcell'+(center?' hcenter':'');
  el.innerHTML=center
    ?'<div class="hd" style="font-size:11px;opacity:.3">center</div>'
    :'<div class="hn"></div><div class="hd">--</div>';
  hmap.appendChild(el);hcells.push(el);
}

// ── build channel grid (toggles + last cmd) ──
const cgrid=document.getElementById('cgrid');
const ctoggles=[];
for(let r=0;r<3;r++)for(let c=0;c<3;c++){
  const el=document.createElement('div');
  const center=r===1&&c===1;
  el.className='ccell'+(center?' ccenter':'');
  if(center){el.innerHTML='<div style="text-align:center;opacity:.2;font-size:11px;margin:auto">center</div>';}
  else{
    // find channel index for this position
    const i=CH_POS.findIndex(([pr,pc])=>pr===r&&pc===c);
    el.innerHTML=`<div><div class="cn">ch${i} ${CH_NAME[i]}</div><div class="cs" id="cs${i}">disabled</div><div class="ccmd" id="ccmd${i}">--</div></div>
      <label class="toggle"><input type="checkbox" id="ct${i}" onchange="toggleCh(${i})"><span class="slider"></span></label>`;
    ctoggles[i]=el.querySelector('input');
  }
  cgrid.appendChild(el);
}

async function toggleCh(i){await fetch('/toggle/channel/'+i,{method:'POST'})}
async function toggleAll(){await fetch('/toggle/all',{method:'POST'})}
async function toggleCamera(){await fetch('/toggle/camera',{method:'POST'})}

async function update(){
  try{
    const d=await(await fetch('/data')).json();
    // heatmap
    for(let i=0;i<8;i++){
      const[r,c]=CH_POS[i];
      const el=hcells[r*3+c];
      const ch=d.channels[i];
      el.style.background=distColor(ch.dist,ch.enabled);
      el.querySelector('.hn').textContent=`ch${i} ${CH_NAME[i]}`;
      el.querySelector('.hd').textContent=!ch.enabled?'OFF':ch.dist>=1?'--':ch.dist.toFixed(2)+'m';
    }
    // channel grid
    for(let i=0;i<8;i++){
      const ch=d.channels[i];
      const [r,c]=CH_POS[i];
      const cell=cgrid.children[r*3+c];
      cell.className='ccell'+(ch.enabled?'':' cdisabled');
      ctoggles[i].checked=ch.enabled;
      document.getElementById('cs'+i).textContent=
        ch.enabled?(ch.running?'● running':'○ stopped'):'disabled';
      document.getElementById('ccmd'+i).textContent=ch.cmd||'--';
    }
    // master
    const anyOn=d.channels.some(c=>c.enabled);
    const allOn=d.channels.every(c=>c.enabled);
    document.getElementById('masterTog').checked=anyOn;
    document.getElementById('masterSub').textContent=allOn?'all on':anyOn?'partial':'all off';
    // camera
    const btn=document.getElementById('camBtn');
    btn.textContent='Camera: '+(d.camera?'ON':'OFF');
    btn.className='btn '+(d.camera?'bon':'boff');
  }catch(e){}
}
setInterval(update,300);update();
</script>
</body>
</html>
)=====";

// ── Web handlers ──────────────────────────────────────────────────────────────

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  String j = "{\"grid\":[";
  for (int r = 0; r < 3; r++) {
    j += "[";
    for (int c = 0; c < 3; c++) {
      j += String(lastGrid[r][c], 2);
      if (c < 2) j += ",";
    }
    j += "]";
    if (r < 2) j += ",";
  }
  j += "],\"channels\":[";
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    j += "{\"enabled\":" + String(chEnabled[i] ? "true" : "false");
    j += ",\"running\":"  + String(chRunning[i] ? "true" : "false");
    j += ",\"dist\":"     + String(lastDist[i], 2);
    j += ",\"cmd\":\""   + String(lastCmd[i]) + "\"}";
    if (i < CHANNEL_COUNT - 1) j += ",";
  }
  j += "],\"camera\":" + String(cameraEnabled ? "true" : "false") + "}";
  server.send(200, "application/json", j);
}

void enableChannel(uint8_t ch) {
  chEnabled[ch] = true;
  switchChannel(ch);
  sendEMSCmd("FE 00 00 01 100", ch);
  sendEMSCmd("start_ems", ch);
  chRunning[ch] = true;
}

void disableChannel(uint8_t ch) {
  chEnabled[ch] = false;
  if (chRunning[ch]) {
    switchChannel(ch);
    sendEMSCmd("stop_ems", ch);
    chRunning[ch] = false;
  }
}

void handleToggleChannel() {
  String uri = server.uri();  // /toggle/channel/N
  int ch = uri.substring(uri.lastIndexOf('/') + 1).toInt();
  if (ch < 0 || ch >= CHANNEL_COUNT) { server.send(400); return; }
  if (chEnabled[ch]) disableChannel(ch); else enableChannel(ch);
  server.send(200, "text/plain", "ok");
}

void handleToggleAll() {
  // if any channel is enabled → disable all; otherwise enable all
  bool anyOn = false;
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) if (chEnabled[i]) { anyOn = true; break; }
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    if (anyOn) disableChannel(i); else enableChannel(i);
  }
  server.send(200, "text/plain", "ok");
}

void handleToggleCamera() {
  cameraEnabled = !cameraEnabled;
  if (cameraEnabled) {
    Camera_UART.print("AT+ISP=1\r");
  } else {
    Camera_UART.print("AT+ISP=0\r");
    delay(100);
    while (Camera_UART.available()) Camera_UART.read();
  }
  server.send(200, "text/plain", "ok");
}

// ── Mux ──────────────────────────────────────────────────────────────────────

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
  strncpy(lastCmd[ch], cmd, sizeof(lastCmd[ch]) - 1);
  EMS_UART.print(cmd);
  EMS_UART.print("\n");
  uint32_t t0 = millis();
  while (millis() - t0 < SEND_INTERVAL_MS) {
    if (EMS_UART.available()) Serial.write(EMS_UART.read());
  }
}

void initAllChannels() {
  Serial.println("init channels");
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    if (!chEnabled[ch]) continue;
    switchChannel(ch);
    sendEMSCmd("FE 00 00 01 100", ch);
    sendEMSCmd("start_ems", ch);
    chRunning[ch] = true;
  }
}

int distToVal(float dist_m) {
  if (dist_m > 1.0f) return 0;
  int v = (int)round(5.0f - dist_m * 4.0f);
  if (v < 1) v = 1;
  if (v > 5) v = 5;
  return v;
}

void updateEMSChannel(uint8_t ch, float dist_m) {
  if (!chEnabled[ch]) return;
  switchChannel(ch);
  lastDist[ch] = dist_m;
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

float pixelToMeters(uint8_t p) {
  float d = p / 5.1f;
  return (d * d) / 1000.0f;
}

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
      uint8_t pixels[81];
      uint8_t cnt = 0;
      for (uint8_t row = BLK_START[r]; row < BLK_END[r]; row++)
        for (uint8_t col = BLK_START[c]; col < BLK_END[c]; col++)
          pixels[cnt++] = buf[row * 25 + col];
      sortBytes(pixels, cnt);
      uint8_t trim = cnt / 10;
      uint32_t sum = 0;
      for (uint8_t i = trim; i < cnt - trim; i++) sum += pixels[i];
      grid[r][c] = (float)sum / (cnt - 2 * trim);
    }
  }
}

bool readCameraFrame(uint8_t* buf) {
  uint32_t deadline = millis() + 500;
  uint8_t prev = 0xFF;
  bool synced = false;
  while (millis() < deadline) {
    if (!Camera_UART.available()) continue;
    uint8_t b = Camera_UART.read();
    if (prev == 0x00 && b == 0xFF) { synced = true; break; }
    prev = b;
  }
  if (!synced) return false;
  for (uint8_t i = 0; i < 18; ) {
    if (millis() >= deadline) return false;
    if (Camera_UART.available()) { Camera_UART.read(); i++; }
  }
  for (uint16_t i = 0; i < 625; ) {
    if (millis() >= deadline) return false;
    if (Camera_UART.available()) { buf[i++] = Camera_UART.read(); }
  }
  for (uint8_t i = 0; i < 2; ) {
    if (millis() >= deadline) return false;
    if (Camera_UART.available()) { Camera_UART.read(); i++; }
  }
  return true;
}

// ── Manual serial commands ────────────────────────────────────────────────────

void pollEMS() {
  for (int i = 0; i < 20; i++) {
    EMS_UART.print("FE 00 00 01 30\n");
    uint32_t start = millis();
    bool printed = false;
    while (millis() - start < 500) {
      if (EMS_UART.available()) {
        if (!printed) { Serial.print("["); Serial.print(i+1); Serial.print("] "); printed = true; }
        Serial.write(EMS_UART.read());
      }
    }
  }
}

void runAutoTest(uint32_t switchWait, uint32_t sendInterval) {
  Serial.print("\n=== autotest switchWait="); Serial.print(switchWait);
  Serial.print("ms sendInterval="); Serial.print(sendInterval); Serial.println("ms ===");
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    Serial.print("\n--- ch"); Serial.print(ch); Serial.println(" ---");
    uint32_t t0 = millis(); switchChannel(ch);
    Serial.print("switch: "); Serial.print(millis()-t0); Serial.println("ms");
    delay(switchWait);
    for (int i = 0; i < 10; i++) {
      uint32_t s = millis();
      EMS_UART.print("FE 00 00 01 30\n");
      bool got = false, pr = false;
      while (millis()-s < sendInterval) {
        if (EMS_UART.available()) {
          if (!pr) { Serial.print("["); Serial.print(i+1); Serial.print("] +");
                     Serial.print(millis()-s); Serial.print("ms: "); pr=true; got=true; }
          Serial.write(EMS_UART.read());
        }
      }
      Serial.print("  interval: "); Serial.print(millis()-s); Serial.println("ms");
      if (!got) { Serial.print("["); Serial.print(i+1); Serial.println("] no response -> stop"); return; }
    }
  }
  Serial.println("\n=== autotest done ===");
}

bool handleSerialCommand() {
  if (serialLineLength == 1 &&
      serialLineBuffer[0] >= '0' && serialLineBuffer[0] < ('0' + CHANNEL_COUNT)) {
    uint8_t ch = serialLineBuffer[0] - '0';
    switchChannel(ch); Serial.print("selected "); Serial.println(ch); pollEMS(); return true;
  }
  if (serialLineLength == 8 &&
      serialLineBuffer[0]=='s' && serialLineBuffer[1]=='e' && serialLineBuffer[2]=='l' &&
      serialLineBuffer[3]=='e' && serialLineBuffer[4]=='c' && serialLineBuffer[5]=='t' &&
      serialLineBuffer[6]==' ' &&
      serialLineBuffer[7] >= '0' && serialLineBuffer[7] < ('0' + CHANNEL_COUNT)) {
    uint8_t ch = serialLineBuffer[7] - '0';
    switchChannel(ch); Serial.print("selected "); Serial.println(ch); pollEMS(); return true;
  }
  if (serialLineLength >= 8 &&
      serialLineBuffer[0]=='a' && serialLineBuffer[1]=='u' && serialLineBuffer[2]=='t' &&
      serialLineBuffer[3]=='o' && serialLineBuffer[4]=='t' && serialLineBuffer[5]=='e' &&
      serialLineBuffer[6]=='s' && serialLineBuffer[7]=='t') {
    uint32_t sw = 1000, si = 500;
    if (serialLineLength > 9) sw = atoi(serialLineBuffer + 9);
    uint8_t sp = 0;
    for (uint8_t k = 9; k < serialLineLength; k++) if (serialLineBuffer[k]==' ') { sp=k; break; }
    if (sp > 0 && sp+1 < serialLineLength) si = atoi(serialLineBuffer + sp + 1);
    runAutoTest(sw, si); return true;
  }
  return false;
}

void flushBufferedSerialToEMS() {
  for (uint8_t i = 0; i < serialLineLength; ++i) EMS_UART.write(serialLineBuffer[i]);
}

// ── Setup / Loop ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  EMS_UART.begin(115200, SERIAL_8N1, 17, 18);
  Camera_UART.begin(115200);

  FastLED.addLeds<SK6812, LED_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Blue; FastLED.show();

  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_C, OUTPUT);
  selectChannel(0);

  // init channel state
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    chEnabled[i] = false;
    chRunning[i] = false;
    lastDist[i] = 9.9f;
    lastCmd[i][0] = '\0';
  }

  // WiFi AP
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  // web routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/toggle/camera", HTTP_POST, handleToggleCamera);
  server.on("/toggle/all",    HTTP_POST, handleToggleAll);
  for (int i = 0; i < CHANNEL_COUNT; i++)
    server.on(("/toggle/channel/" + String(i)).c_str(), HTTP_POST, handleToggleChannel);
  server.begin();
  Serial.println("web server started");

  // configure camera
  delay(500);
  Camera_UART.print("AT+FPS=10\r");  delay(200); while (Camera_UART.available()) Camera_UART.read();
  Camera_UART.print("AT+BINN=4\r");  delay(200); while (Camera_UART.available()) Camera_UART.read();
  Camera_UART.print("AT+DISP=5\r");  delay(2000); while (Camera_UART.available()) Camera_UART.read();
  Serial.println("camera configured");

  leds[0] = CRGB::Green; FastLED.show();
  Serial.println("ready");
}

void loop() {
  server.handleClient();

  // USB serial passthrough / commands
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\r' || ch == '\n') {
      if (serialLineLength > 0) {
        if (!handleSerialCommand()) { flushBufferedSerialToEMS(); EMS_UART.write(ch); }
        serialLineLength = 0;
      }
      continue;
    }
    if (serialLineLength < sizeof(serialLineBuffer) - 1) {
      serialLineBuffer[serialLineLength++] = ch;
      serialLineBuffer[serialLineLength] = '\0';
    } else {
      flushBufferedSerialToEMS(); EMS_UART.write(ch); serialLineLength = 0;
    }
  }

  if (!cameraEnabled) { server.handleClient(); return; }

  // read one ToF frame
  static uint8_t frameBuf[625];
  if (!readCameraFrame(frameBuf)) {
    Serial.println("frame timeout");
    leds[0] = CRGB::Red; FastLED.show();
    return;
  }
  leds[0] = CRGB::Green; FastLED.show();

  // downscale 25x25 → 3x3
  float grid[3][3];
  computeGrid(frameBuf, grid);
  memcpy(lastGrid, grid, sizeof(grid));

  // apply center override: if center is closer, propagate to surrounding cells
  float centerDist = pixelToMeters((uint8_t)grid[1][1]);
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    float d = pixelToMeters((uint8_t)grid[CH_ROW[ch]][CH_COL[ch]]);
    if (centerDist < d) grid[CH_ROW[ch]][CH_COL[ch]] = grid[1][1];
  }

  // print 3x3
  Serial.println("grid (m):");
  for (uint8_t r = 0; r < 3; r++) {
    for (uint8_t c = 0; c < 3; c++) {
      if (r == 1 && c == 1) { Serial.print("  --  "); continue; }
      char tmp[8]; dtostrf(pixelToMeters((uint8_t)grid[r][c]), 4, 2, tmp);
      Serial.print(tmp); Serial.print(" ");
    }
    Serial.println();
  }

  // update all 8 EMS channels
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    float dist_m = pixelToMeters((uint8_t)grid[CH_ROW[ch]][CH_COL[ch]]);
    updateEMSChannel(ch, dist_m);
  }

  server.handleClient();
}
