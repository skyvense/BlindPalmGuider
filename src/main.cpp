#include <HardwareSerial.h>
#include <FastLED.h>
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

#define WIFI_SSID "S1"
#define WIFI_PASS "checkin888"

CRGB leds[NUM_LEDS];
HardwareSerial EMS_UART(2);
HardwareSerial Camera_UART(1);
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
float   lastDist[CHANNEL_COUNT] = {9.9f};
char    lastCmd[CHANNEL_COUNT][32] = {""};
float   lastGrid[3][3] = {{}};
bool    cameraEnabled = true;
uint8_t chMin[CHANNEL_COUNT];
uint8_t chMax[CHANNEL_COUNT];
float   distThreshold = 1.0f;  // max distance in meters (0.1–3.0)
uint8_t viewRotation  = 0;     // 0=0° 1=90°CW 2=180° 3=270°CW

// 3x3 grid → channel (skip center [1][1])
// ch: 0=TL 1=T 2=TR 3=L 4=R 5=BL 6=B 7=BR
static const uint8_t CH_ROW[CHANNEL_COUNT] = {0, 0, 0, 1, 1, 2, 2, 2};
static const uint8_t CH_COL[CHANNEL_COUNT] = {0, 1, 2, 0, 2, 0, 1, 2};

// Map visual (r,c) → source camera (r,c) given rotation
static void vToSrc(uint8_t r, uint8_t c, uint8_t rot, uint8_t &sr, uint8_t &sc) {
  switch (rot) {
    case 1: sr = 2 - c; sc = r;     return;
    case 2: sr = 2 - r; sc = 2 - c; return;
    case 3: sr = c;     sc = 2 - r; return;
    default: sr = r;    sc = c;
  }
}

static const uint8_t BLK_START[3] = {0,  8, 17};
static const uint8_t BLK_END[3]   = {8, 17, 25};

// ── Web page ──────────────────────────────────────────────────────────────────

static const char INDEX_HTML[] PROGMEM = R"~HTML~(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>EMS Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;font-family:sans-serif}
body{background:#1a1a2e;color:#eee;padding:14px;max-width:480px;margin:0 auto}
.sec{font-size:11px;opacity:.4;text-transform:uppercase;letter-spacing:1px;margin:14px 0 6px}
.g3{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-bottom:6px}
.hcell{border-radius:8px;padding:10px 4px;text-align:center;transition:background .35s;min-height:66px;display:flex;flex-direction:column;justify-content:center}
.hn{font-size:10px;opacity:.55;margin-bottom:2px}
.hd{font-size:19px;font-weight:700}
.hcenter{background:#222236!important}
.ccell{border-radius:8px;padding:7px 6px;background:#16213e;min-height:86px;display:flex;flex-direction:column;justify-content:space-between}
.ccell.off{opacity:.38}
.ccenter2{background:#1a1a2e!important;border:1px dashed #2a2a4a}
.cn{font-size:11px;font-weight:700}
.cs{font-size:9px;opacity:.5;margin:2px 0 3px}
.ccmd{font-size:8px;opacity:.65;word-break:break-all;flex:1;margin-bottom:4px;line-height:1.3}
.tog{position:relative;width:40px;height:22px;flex-shrink:0;align-self:flex-end}
.tog input{opacity:0;width:0;height:0}
.sl{position:absolute;cursor:pointer;inset:0;background:#444;border-radius:22px;transition:.25s}
.sl:before{position:absolute;content:"";height:16px;width:16px;left:3px;bottom:3px;background:#fff;border-radius:50%;transition:.25s}
input:checked+.sl{background:#43a047}
input:checked+.sl:before{transform:translateX(18px)}
.rotbar{display:flex;align-items:center;gap:8px;margin-bottom:6px}
.rotbar span{flex:1;font-size:12px;opacity:.65}
.rbtn{background:#0f3460;border:none;color:#eee;border-radius:6px;padding:5px 14px;font-size:15px;cursor:pointer}
/* dual range */
.dr-row{background:#16213e;border-radius:8px;padding:8px 12px;margin-bottom:6px}
.dr-lbl{font-size:12px;display:flex;justify-content:space-between;margin-bottom:8px}
.dr-lbl span{opacity:.55;font-size:11px}
.dr{position:relative;height:22px;display:flex;align-items:center}
.dr-track{position:absolute;left:0;right:0;height:4px;background:#252538;border-radius:2px}
.dr-fill{position:absolute;height:4px;background:#43a047;border-radius:2px;pointer-events:none}
.dr input[type=range]{position:absolute;width:100%;pointer-events:none;-webkit-appearance:none;appearance:none;background:transparent;height:4px;margin:0}
.dr input::-webkit-slider-thumb{pointer-events:all;width:16px;height:16px;border-radius:50%;background:#43a047;border:2px solid #fff;cursor:pointer;-webkit-appearance:none;box-shadow:0 1px 4px #0006}
.dr input::-moz-range-thumb{pointer-events:all;width:14px;height:14px;border-radius:50%;background:#43a047;border:2px solid #fff;cursor:pointer;box-shadow:0 1px 4px #0006}
/* threshold */
.trow{background:#16213e;border-radius:8px;padding:9px 12px;margin-bottom:6px;display:flex;align-items:center;gap:8px}
.trow input[type=range]{flex:1;accent-color:#e57c1e;height:4px}
.tv{font-size:13px;width:46px;text-align:right;color:#e57c1e;font-weight:700;flex-shrink:0}
/* controls */
.crow{display:flex;justify-content:space-between;align-items:center;padding:10px 14px;background:#16213e;border-radius:8px;margin-bottom:8px}
.crow b{font-size:14px}
.crow small{font-size:11px;opacity:.5;display:block;margin-top:2px}
.btn{width:100%;padding:12px;border:none;color:#fff;border-radius:8px;font-size:14px;cursor:pointer;margin-bottom:8px;transition:.25s}
.bon{background:#0d47a1}.boff{background:#555}
.master{border:1px solid #43a047}
</style>
</head>
<body>

<div class="sec">Depth Heatmap</div>
<div class="rotbar">
  <span id="rotLbl">Rotation: 0°</span>
  <button class="rbtn" onclick="rotate(-1)">↺ 90°</button>
  <button class="rbtn" onclick="rotate(1)">↻ 90°</button>
</div>
<div class="g3" id="hmap"></div>

<div class="sec">Channels</div>
<div class="g3" id="cgrid"></div>

<div class="sec">Distance Threshold</div>
<div class="trow">
  <span style="font-size:11px;opacity:.5">0.1m</span>
  <input type="range" id="thrSlider" min="10" max="300" step="5" value="100" oninput="onThr(this.value)">
  <span style="font-size:11px;opacity:.5">3.0m</span>
  <span class="tv" id="thrVal">1.00m</span>
</div>

<div class="sec">Intensity Range (1 – 30)</div>
<div id="ranges"></div>

<div class="crow master" style="margin-top:6px">
  <div><b>All Channels</b><small id="masterSub">all off</small></div>
  <label class="tog"><input type="checkbox" id="masterTog" onchange="toggleAll()"><span class="sl"></span></label>
</div>
<button class="btn bon" id="camBtn" onclick="toggleCamera()">Camera: ON</button>

<script>
const CH_NAME=['TL','T','TR','L','R','BL','B','BR'];
const CH_POS=[[0,0],[0,1],[0,2],[1,0],[1,2],[2,0],[2,1],[2,2]];
let rotation=0,lastThr=1.0;

function vToSrc(r,c,rot){
  switch(rot){case 1:return[2-c,r];case 2:return[2-r,2-c];case 3:return[c,2-r];}return[r,c];
}
function chAt(r,c){
  if(r===1&&c===1)return -1;
  const[sr,sc]=vToSrc(r,c,rotation);
  if(sr===1&&sc===1)return -1;
  return CH_POS.findIndex(([pr,pc])=>pr===sr&&pc===sc);
}
function distColor(dist,thr){
  if(dist>thr)return'#1a2a4a';
  const t=1-dist/thr;
  return`rgb(${Math.round(t*230)},${Math.round((0.5-Math.abs(t-.5))*200)},${Math.round((1-t)*160)})`;
}

// ── heatmap & channel grids ──
const hmap=document.getElementById('hmap');
const cgrid=document.getElementById('cgrid');
const hcells=[],ccells=[],ctoggles=new Array(8);
for(let r=0;r<3;r++)for(let c=0;c<3;c++){
  const center=r===1&&c===1;
  const h=document.createElement('div');
  h.className='hcell'+(center?' hcenter':'');
  h.innerHTML=center?'<div class="hd" style="font-size:10px;opacity:.2">·</div>'
                    :'<div class="hn"></div><div class="hd">--</div>';
  hmap.appendChild(h);hcells.push(h);
  const cc=document.createElement('div');
  cc.className=center?'ccell ccenter2':'ccell';
  cc.innerHTML=center?'<div style="margin:auto;opacity:.12;font-size:10px">center</div>':'';
  cgrid.appendChild(cc);ccells.push(cc);
}

function rebuildCells(){
  for(let r=0;r<3;r++)for(let c=0;c<3;c++){
    if(r===1&&c===1)continue;
    const i=chAt(r,c),cc=ccells[r*3+c];
    cc.innerHTML=
      `<div><div class="cn">ch${i} ${CH_NAME[i]}</div>`+
      `<div class="cs" id="cs${i}">--</div>`+
      `<div class="ccmd" id="ccmd${i}">--</div></div>`+
      `<label class="tog"><input type="checkbox" id="ct${i}" onchange="toggleCh(${i})"><span class="sl"></span></label>`;
    ctoggles[i]=cc.querySelector('input');
  }
}
rebuildCells();

function rotate(dir){
  rotation=(rotation+dir+4)%4;
  document.getElementById('rotLbl').textContent='Rotation: '+['0°','90°','180°','270°'][rotation];
  rebuildCells();
  fetch(`/set/rotation?v=${rotation}`,{method:'POST'});
}

// ── dual range sliders ──
const rangesDiv=document.getElementById('ranges');
for(let i=0;i<8;i++){
  const d=document.createElement('div');d.className='dr-row';
  d.innerHTML=
    `<div class="dr-lbl"><b>ch${i} ${CH_NAME[i]}</b><span id="drv${i}">1 – 5</span></div>`+
    `<div class="dr">`+
      `<div class="dr-track"></div><div class="dr-fill" id="drf${i}"></div>`+
      `<input type="range" id="drn${i}" min="1" max="30" value="1" oninput="drMove(${i})">`+
      `<input type="range" id="drx${i}" min="1" max="30" value="5" oninput="drMove(${i})">`+
    `</div>`;
  rangesDiv.appendChild(d);drFill(i);
}
function drFill(i){
  const mn=+document.getElementById('drn'+i).value;
  const mx=+document.getElementById('drx'+i).value;
  const p1=(mn-1)/29*100,p2=(mx-1)/29*100;
  const f=document.getElementById('drf'+i);
  f.style.left=p1+'%';f.style.width=(p2-p1)+'%';
  document.getElementById('drv'+i).textContent=mn+' – '+mx;
}
const drT={};
function drMove(i){
  let mn=+document.getElementById('drn'+i).value;
  let mx=+document.getElementById('drx'+i).value;
  if(mn>mx){document.getElementById('drn'+i).value=mx;mn=mx;}
  drFill(i);
  clearTimeout(drT[i]);
  drT[i]=setTimeout(()=>fetch(`/set/range/${i}?min=${mn}&max=${mx}`,{method:'POST'}),300);
}

// ── threshold ──
let thrT;
function onThr(v){
  const m=v/100;
  document.getElementById('thrVal').textContent=m.toFixed(2)+'m';
  clearTimeout(thrT);
  thrT=setTimeout(()=>fetch(`/set/threshold?v=${m}`,{method:'POST'}),300);
}

async function toggleCh(i){await fetch('/toggle/channel/'+i,{method:'POST'})}
async function toggleAll(){await fetch('/toggle/all',{method:'POST'})}
async function toggleCamera(){await fetch('/toggle/camera',{method:'POST'})}

async function update(){
  try{
    const d=await(await fetch('/data')).json();
    lastThr=d.threshold||1;
    // heatmap (rotated)
    for(let r=0;r<3;r++)for(let c=0;c<3;c++){
      if(r===1&&c===1)continue;
      const i=chAt(r,c);if(i<0)continue;
      const ch=d.channels[i],el=hcells[r*3+c];
      el.style.background=distColor(ch.dist,lastThr);
      el.querySelector('.hn').textContent=`ch${i} ${CH_NAME[i]}`;
      el.querySelector('.hd').textContent=ch.dist>lastThr?'--':ch.dist.toFixed(2)+'m';
    }
    // channel cells (rotated)
    for(let i=0;i<8;i++){
      const ch=d.channels[i],el=document.getElementById('cs'+i);
      if(!el)continue;
      const[r,c]=CH_POS[i];
      ccells[r*3+c].className='ccell'+(ch.enabled?'':' off');
      if(ctoggles[i])ctoggles[i].checked=ch.enabled;
      el.textContent=ch.enabled?(ch.running?'● running':'○ stopped'):'disabled';
      document.getElementById('ccmd'+i).textContent=ch.cmd||'--';
    }
    // range sliders
    const act=document.activeElement?.id||'';
    if(!act.match(/^dr[nx]/)){
      for(let i=0;i<8;i++){
        const ch=d.channels[i];
        document.getElementById('drn'+i).value=ch.min;
        document.getElementById('drx'+i).value=ch.max;
        drFill(i);
      }
    }
    // threshold
    if(!act.match(/^thr/)){
      document.getElementById('thrSlider').value=Math.round(lastThr*100);
      document.getElementById('thrVal').textContent=lastThr.toFixed(2)+'m';
    }
    // master
    const anyOn=d.channels.some(c=>c.enabled);
    document.getElementById('masterTog').checked=anyOn;
    document.getElementById('masterSub').textContent=
      d.channels.every(c=>c.enabled)?'all on':anyOn?'partial':'all off';
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
)~HTML~";
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
    j += ",\"cmd\":\""   + String(lastCmd[i]) + "\"";
    j += ",\"min\":"      + String(chMin[i]);
    j += ",\"max\":"      + String(chMax[i]) + "}";
    if (i < CHANNEL_COUNT - 1) j += ",";
  }
  j += "],\"camera\":"     + String(cameraEnabled ? "true" : "false");
  j += ",\"threshold\":"  + String(distThreshold, 2);
  j += ",\"rotation\":"   + String(viewRotation) + "}";
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

void handleSetRotation() {
  if (server.hasArg("v")) {
    int v = server.arg("v").toInt();
    viewRotation = (uint8_t)constrain(v, 0, 3);
  }
  server.send(200, "text/plain", "ok");
}

void handleSetThreshold() {
  if (server.hasArg("v")) {
    float v = server.arg("v").toFloat();
    distThreshold = constrain(v, 0.1f, 3.0f);
  }
  server.send(200, "text/plain", "ok");
}

void handleSetRange() {
  String uri = server.uri();  // /set/range/N
  int ch = uri.substring(uri.lastIndexOf('/') + 1).toInt();
  if (ch < 0 || ch >= CHANNEL_COUNT) { server.send(400); return; }
  if (server.hasArg("min")) {
    int v = server.arg("min").toInt();
    chMin[ch] = constrain(v, 1, 30);
  }
  if (server.hasArg("max")) {
    int v = server.arg("max").toInt();
    chMax[ch] = constrain(v, 1, 30);
  }
  if (chMin[ch] > chMax[ch]) chMin[ch] = chMax[ch];
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
    else yield();
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

int distToVal(float dist_m, uint8_t ch) {
  if (dist_m > distThreshold) return 0;
  float t = 1.0f - dist_m / distThreshold;  // 0=far, 1=near
  int v = (int)round(chMin[ch] + t * (chMax[ch] - chMin[ch]));
  if (v < (int)chMin[ch]) v = chMin[ch];
  if (v > (int)chMax[ch]) v = chMax[ch];
  return v;
}

void updateEMSChannel(uint8_t ch, float dist_m) {
  lastDist[ch] = dist_m;
  if (!chEnabled[ch]) return;
  switchChannel(ch);
  int val = distToVal(dist_m, ch);

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
    if (!Camera_UART.available()) { yield(); continue; }
    uint8_t b = Camera_UART.read();
    if (prev == 0x00 && b == 0xFF) { synced = true; break; }
    prev = b;
  }
  if (!synced) return false;
  for (uint8_t i = 0; i < 18; ) {
    if (millis() >= deadline) return false;
    if (Camera_UART.available()) { Camera_UART.read(); i++; }
    else yield();
  }
  for (uint16_t i = 0; i < 625; ) {
    if (millis() >= deadline) return false;
    if (Camera_UART.available()) { buf[i++] = Camera_UART.read(); }
    else yield();
  }
  for (uint8_t i = 0; i < 2; ) {
    if (millis() >= deadline) return false;
    if (Camera_UART.available()) { Camera_UART.read(); i++; }
    else yield();
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
      } else yield();
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
        } else yield();
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
  Camera_UART.begin(115200, SERIAL_8N1, 8, 9);

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
    lastDist[i]  = 9.9f;
    lastCmd[i][0] = '\0';
    chMin[i] = 1;
    chMax[i] = 5;
  }
  delay(5000);
  // WiFi STA
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: "); Serial.println(WiFi.localIP());

  // web routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/toggle/camera", HTTP_POST, handleToggleCamera);
  server.on("/toggle/all",    HTTP_POST, handleToggleAll);
  server.on("/set/threshold", HTTP_POST, handleSetThreshold);
  server.on("/set/rotation",  HTTP_POST, handleSetRotation);
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    server.on(("/toggle/channel/" + String(i)).c_str(), HTTP_POST, handleToggleChannel);
    server.on(("/set/range/"      + String(i)).c_str(), HTTP_POST, handleSetRange);
  }
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

  // apply center override using rotated source coordinates
  float centerDist = pixelToMeters((uint8_t)grid[1][1]);
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    uint8_t sr, sc;
    vToSrc(CH_ROW[ch], CH_COL[ch], viewRotation, sr, sc);
    float d = pixelToMeters((uint8_t)grid[sr][sc]);
    if (centerDist < d) grid[sr][sc] = grid[1][1];
  }

  // update all 8 EMS channels using rotated source coordinates
  for (uint8_t ch = 0; ch < CHANNEL_COUNT; ch++) {
    uint8_t sr, sc;
    vToSrc(CH_ROW[ch], CH_COL[ch], viewRotation, sr, sc);
    float dist_m = pixelToMeters((uint8_t)grid[sr][sc]);
    updateEMSChannel(ch, dist_m);
  }

  server.handleClient();
}
