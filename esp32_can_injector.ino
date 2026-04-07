#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

#include <SPI.h>
#include <mcp_can.h>

// =================== USER CONFIG ===================
static const char* AP_SSID = "ESP32-CAN-INJECTOR";
static const char* AP_PASS = ""; // open AP; if set, must be >= 8 chars

static const uint8_t PIN_CS  = 5;
static const uint8_t PIN_INT = 4;

// Set these to match your CAN bus
static const uint32_t CAN_BITRATE = 500000;

// HW-184 commonly has 8MHz crystal; some are 16MHz.
// If CAN init fails, toggle this.
static const bool MCP2515_OSC_8MHZ = true;
// ===================================================

WebServer server(80);
WebSocketsServer ws(81); // ws://192.168.4.1:81/

MCP_CAN CAN(PIN_CS);

static bool g_globalEnable = true;

// ---------------- TX Scheduler ----------------
struct TxFrame {
  bool used = false;
  uint32_t id = 0;
  bool ext = false;
  uint8_t dlc = 8;
  uint8_t data[8] = {0};
  uint32_t periodMs = 100;
  bool enabled = false;
  uint32_t nextDue = 0;
};

static const int MAX_FRAMES = 128;
TxFrame frames[MAX_FRAMES];

int findFrame(uint32_t id, bool ext) {
  for (int i = 0; i < MAX_FRAMES; i++) {
    if (frames[i].used && frames[i].id == id && frames[i].ext == ext) return i;
  }
  return -1;
}

int allocFrame(uint32_t id, bool ext) {
  int idx = findFrame(id, ext);
  if (idx >= 0) return idx;

  for (int i = 0; i < MAX_FRAMES; i++) {
    if (!frames[i].used) {
      frames[i].used = true;
      frames[i].id = id;
      frames[i].ext = ext;
      frames[i].dlc = 8;
      memset(frames[i].data, 0, 8);
      frames[i].periodMs = 100;
      frames[i].enabled = false;
      frames[i].nextDue = millis() + frames[i].periodMs;
      return i;
    }
  }
  return -1;
}

void clearAllFrames() {
  for (int i = 0; i < MAX_FRAMES; i++) frames[i] = TxFrame{};
}

// ---------------- Web UI HTML ----------------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>ESP32 CAN Injector</title>
  <style>
    body{font-family:system-ui,sans-serif;margin:16px;max-width:1100px}
    .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
    .pill{padding:2px 10px;border-radius:999px;background:#eee;font-size:12px}
    .ok{color:#166534}.bad{color:#b91c1c}
    table{border-collapse:collapse;width:100%;margin-top:10px}
    th,td{border:1px solid #ddd;padding:6px;font-size:13px;vertical-align:top}
    th{background:#f6f6f6;text-align:left}
    input[type=number]{width:120px}
    .small{font-size:12px;color:#555}
    .warn{color:#b45309}
    code{background:#f6f6f6;padding:2px 5px;border-radius:6px}
    button{padding:6px 10px}
    .statusBar{display:flex;gap:8px;align-items:center}
  </style>
</head>
<body>
  <h2>ESP32 CAN Injector</h2>

  <div class="row statusBar">
    <span class="pill" id="wsState">WS: connecting…</span>

    <button id="btnGlobalEnable">Disable All</button>
    <button id="btnClear">Clear on ESP32</button>
    <button id="btnPushAll">Push all frames now</button>

    <label class="small">Random update tick (ms):
      <input id="tickMs" type="number" min="10" value="200">
    </label>
  </div>

  <p class="small">
    Connect to Wi-Fi <b>ESP32-CAN-INJECTOR</b> → open <code>http://192.168.4.1</code><br/>
    WebSocket: <code>ws://192.168.4.1:81/</code><br/>
    Packs Intel/little-endian signals (@1). Big-endian (@0) signals are skipped.
  </p>

  <div class="row">
    <input id="dbcFile" type="file" accept=".dbc"/>
    <button id="btnLoad">Load DBC</button>
  </div>

  <div id="note" class="small warn"></div>
  <div id="ui"></div>

<script>
let ws;
let db = { messages: [] };

let state = {
  msg: {},   // msgName -> {enabled, period}
  sig: {}    // "Msg.Signal" -> {enabled, mode, manual, last}
};

let tickTimer = null;
let uiRendered = false;

function setWs(text, ok){
  const el = document.getElementById("wsState");
  el.textContent = text;
  el.className = "pill " + (ok ? "ok" : "bad");
}

function wsConnect(){
  ws = new WebSocket(`ws://192.168.4.1:81/`);
  ws.onopen  = () => setWs("WS: connected", true);
  ws.onclose = () => setWs("WS: disconnected", false);
  ws.onerror = () => setWs("WS: error", false);
}

function send(obj){
  if(ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
}

// ---------- DBC parsing (tolerant) ----------
function parseDBC(text){
  const lines = text.split(/\r?\n/);
  const messages = [];
  let current = null;

  // BO_ 256 MSG_NAME: 8 NODE
  const reBO = /^BO_\s+(\d+)\s+([^:]+)\s*:\s*(\d+)/;

  // SG_ SigName : 0|16@1+ (0.1,0) [0|100] "unit"
  // Make [min|max] optional, and allow broader signal names.
  const reSG = /^\s*SG_\s+([^\s]+)\s*(?:m\d+)?\s*:\s*(\d+)\|(\d+)@(\d)([+-])\s+\(([^,]+),([^)]+)\)\s*(?:\[\s*([^|]+)\|\s*([^\]]+)\s*\])?\s*"([^"]*)"/;

  for(const line of lines){
    const mBO = line.match(reBO);
    if (mBO){
      current = {
        id: parseInt(mBO[1],10),
        name: mBO[2].trim(),
        dlc: parseInt(mBO[3],10),
        signals: []
      };
      messages.push(current);
      continue;
    }

    if(!current) continue;

    const mSG = line.match(reSG);
    if (mSG){
      const endian = (mSG[4] === "1") ? "little" : "big";
      const min = (mSG[8] !== undefined) ? parseFloat(mSG[8]) : NaN;
      const max = (mSG[9] !== undefined) ? parseFloat(mSG[9]) : NaN;

      current.signals.push({
        name: mSG[1].trim(),
        startBit: parseInt(mSG[2],10),
        bitLen: parseInt(mSG[3],10),
        endian,
        signed: (mSG[5] === "-"),
        factor: parseFloat(mSG[6]),
        offset: parseFloat(mSG[7]),
        min,
        max,
        unit: (mSG[10] || "")
      });
    }
  }

  return { messages };
}


// ---------- Bit packing (Intel / little-endian) ----------
function clamp(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }

function physToRaw(sig, phys){
  let raw = (phys - sig.offset) / sig.factor;
  raw = Math.round(raw);

  const maxRaw = sig.signed ? (2**(sig.bitLen-1)-1) : (2**sig.bitLen - 1);
  const minRaw = sig.signed ? -(2**(sig.bitLen-1)) : 0;
  raw = clamp(raw, minRaw, maxRaw);

  if(sig.signed && raw < 0) raw = (2**sig.bitLen) + raw; // two's complement
  return raw >>> 0;
}

function setBitsLE(bytes, startBit, bitLen, rawU){
  for(let i=0;i<bitLen;i++){
    const bitVal = (rawU >> i) & 1;
    const bitPos = startBit + i;
    const byteIndex = Math.floor(bitPos / 8);
    const bitIndex  = bitPos % 8;
    if(byteIndex < 0 || byteIndex >= bytes.length) continue;

    if(bitVal) bytes[byteIndex] |= (1 << bitIndex);
    else       bytes[byteIndex] &= ~(1 << bitIndex);
  }
}

function randomInRange(sig){
  let lo = Number.isFinite(sig.min) ? sig.min : 0;
  let hi = Number.isFinite(sig.max) ? sig.max : (lo + 100);
  if(hi < lo){ const t=lo; lo=hi; hi=t; }
  return lo + Math.random() * (hi - lo);
}

// Update only the displayed "Last" value cell (no re-render)
function updateLastCell(key, val){
  const el = document.getElementById(`last-${key}`);
  if(!el) return;
  el.textContent = Number.isFinite(val) ? val.toFixed(3) : String(val);
}

function buildFrame(message){
  const bytes = new Uint8Array(8);

  for(const sig of message.signals){
    if(sig.endian !== "little") continue; // skip big-endian in this build

    const key = `${message.name}.${sig.name}`;
    if(!state.sig[key]) state.sig[key] = { enabled:true, mode:"random", manual:0, last:0 };
    const st = state.sig[key];

    let phys;

    if(!st.enabled){
      phys = 0;
    } else if(st.mode === "manual"){
      phys = st.manual;
      st.last = phys;
    } else if(st.mode === "last"){
      phys = st.last;
    } else { // random
      phys = randomInRange(sig);
      st.last = phys;
    }

    const rawU = physToRaw(sig, phys);
    setBitsLE(bytes, sig.startBit, sig.bitLen, rawU);

    // update only text cell
    updateLastCell(key, st.last);
  }

  return bytes;
}

// ---------- UI rendering (ONE TIME) ----------
function renderOnce(){
  const ui = document.getElementById("ui");
  const note = document.getElementById("note");

  let hasBE = false;
  for(const m of db.messages) for(const s of m.signals) if(s.endian !== "little") hasBE = true;
  note.textContent = hasBE ? "Warning: Big-endian (@0) signals exist; this demo skips them." : "";

  let html = "";
  for(const m of db.messages){
    if(!state.msg[m.name]) state.msg[m.name] = { enabled:true, period:50 };

    // IMPORTANT: IDs must be safe for DOM (replace spaces etc.)
    const msgIdSafe = m.name.replaceAll(/[^a-zA-Z0-9_\\-\\.]/g, "_");

    html += `<h3>${m.name} (0x${m.id.toString(16)})</h3>`;
    html += `<div class="row">
      <label>Enable message <input type="checkbox" id="msgEn-${msgIdSafe}" ${state.msg[m.name].enabled ? "checked":""}></label>
      <label>Period ms <input type="number" id="msgPer-${msgIdSafe}" min="1" value="${state.msg[m.name].period}"></label>
      <button id="push-${msgIdSafe}">Push this frame now</button>
    </div>`;

    html += `<table>
      <tr><th>Signal</th><th>Enable</th><th>Mode</th><th>Manual</th><th>Min..Max</th><th>Unit</th><th>Last</th></tr>`;

    for(const s of m.signals){
      const key = `${m.name}.${s.name}`;
      if(!state.sig[key]) state.sig[key] = { enabled:true, mode:"random", manual:0, last:0 };
      const st = state.sig[key];

      const keySafe = key.replaceAll(/[^a-zA-Z0-9_\\-\\.]/g, "_");

      html += `<tr>
        <td>${s.name}</td>
        <td><input type="checkbox" id="sigEn-${keySafe}" ${st.enabled?"checked":""}></td>
        <td>
          <select id="sigMode-${keySafe}">
            <option value="random" ${st.mode==="random"?"selected":""}>random</option>
            <option value="last" ${st.mode==="last"?"selected":""}>last</option>
            <option value="manual" ${st.mode==="manual"?"selected":""}>manual</option>
          </select>
        </td>
        <td><input type="number" step="any" id="sigMan-${keySafe}" value="${st.manual}"></td>
        <td>${Number.isFinite(s.min)?s.min:""} .. ${Number.isFinite(s.max)?s.max:""}</td>
        <td>${s.unit||""}</td>
        <td id="last-${keySafe}">${Number.isFinite(st.last)?st.last.toFixed(3):st.last}</td>
      </tr>`;
    }

    html += `</table>`;
  }

  ui.innerHTML = html;

  // Attach listeners (do not re-render later)
  for(const m of db.messages){
    const msgIdSafe = m.name.replaceAll(/[^a-zA-Z0-9_\\-\\.]/g, "_");

    const msgEn = document.getElementById(`msgEn-${msgIdSafe}`);
    const msgPer= document.getElementById(`msgPer-${msgIdSafe}`);
    const push  = document.getElementById(`push-${msgIdSafe}`);

    msgEn.addEventListener("change", () => { state.msg[m.name].enabled = msgEn.checked; });
    msgPer.addEventListener("change", () => {
      const v = parseInt(msgPer.value,10);
      state.msg[m.name].period = Math.max(1, isNaN(v) ? 50 : v);
    });
    push.addEventListener("click", () => pushMessage(m));

    for(const s of m.signals){
      const key = `${m.name}.${s.name}`;
      const keySafe = key.replaceAll(/[^a-zA-Z0-9_\\-\\.]/g, "_");

      const sigEn   = document.getElementById(`sigEn-${keySafe}`);
      const sigMode = document.getElementById(`sigMode-${keySafe}`);
      const sigMan  = document.getElementById(`sigMan-${keySafe}`);

      sigEn.addEventListener("change", () => { state.sig[key].enabled = sigEn.checked; });
      sigMode.addEventListener("change", () => { state.sig[key].mode = sigMode.value; });
      sigMan.addEventListener("change", () => {
        const fv = parseFloat(sigMan.value);
        state.sig[key].manual = isNaN(fv) ? 0 : fv;
      });
    }
  }

  uiRendered = true;
}

function pushMessage(messageObj){
  const msgState = state.msg[messageObj.name] || { enabled:true, period:50 };
  const bytes = buildFrame(messageObj);

  send({
    type: "set",
    id: messageObj.id,
    ext: 0,
    dlc: messageObj.dlc,
    period: msgState.period,
    enabled: msgState.enabled ? 1 : 0,
    data: Array.from(bytes)
  });
}

function pushAll(){
  for(const m of db.messages) pushMessage(m);
}

function startTicker(){
  if(tickTimer) clearInterval(tickTimer);
  const tick = Math.max(10, parseInt(document.getElementById("tickMs").value,10) || 200);

  tickTimer = setInterval(() => {
    if(!uiRendered) return;

    for(const m of db.messages){
      pushMessage(m);
    }
  }, tick);
}

// ---------- Buttons ----------
document.getElementById("btnLoad").onclick = async () => {
  const f = document.getElementById("dbcFile").files[0];
  if(!f) return;

  const text = await f.text();
  db = parseDBC(text);

  // init state defaults (random)
  for(const m of db.messages){
    if(!state.msg[m.name]) state.msg[m.name] = { enabled:true, period:50 };
    for(const s of m.signals){
      const key = `${m.name}.${s.name}`;
      if(!state.sig[key]) state.sig[key] = { enabled:true, mode:"random", manual:0, last:0 };
    }
  }

  renderOnce();
  startTicker();
};

document.getElementById("btnClear").onclick = () => send({type:"clear"});
document.getElementById("btnPushAll").onclick = () => pushAll();
document.getElementById("tickMs").onchange = () => startTicker();

document.getElementById("btnGlobalEnable").onclick = () => {
  // Check if anything is currently enabled
  let anyEnabled = false;
  for(const m of db.messages){
    if(state.msg[m.name] && state.msg[m.name].enabled) { anyEnabled = true; break; }
    for(const s of m.signals){
      const key = `${m.name}.${s.name}`;
      if(state.sig[key] && state.sig[key].enabled) { anyEnabled = true; break; }
    }
    if(anyEnabled) break;
  }

  const newState = !anyEnabled;

  for(const m of db.messages){
    const msgIdSafe = m.name.replaceAll(/[^a-zA-Z0-9_\\-\\.]/g, "_");

    // Toggle message checkbox
    if(state.msg[m.name]) state.msg[m.name].enabled = newState;
    const msgCb = document.getElementById(`msgEn-${msgIdSafe}`);
    if(msgCb) msgCb.checked = newState;

    // Toggle all signal checkboxes
    for(const s of m.signals){
      const key = `${m.name}.${s.name}`;
      if(state.sig[key]) state.sig[key].enabled = newState;
      const keySafe = key.replaceAll(/[^a-zA-Z0-9_\\-\\.]/g, "_");
      const sigCb = document.getElementById(`sigEn-${keySafe}`);
      if(sigCb) sigCb.checked = newState;
    }
  }

  pushAll();

  document.getElementById("btnGlobalEnable").innerText =
    newState ? "Disable All" : "Enable All";
};

wsConnect();
</script>
</body>
</html>
)rawliteral";

// ---------------- HTTP + WS handlers ----------------
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void wsEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("WS client %u connected\n", clientNum);
    return;
  }
  if (type == WStype_DISCONNECTED) {
    Serial.printf("WS client %u disconnected\n", clientNum);
    return;
  }
  if (type != WStype_TEXT) return;

  StaticJsonDocument<768> doc;
  auto err = deserializeJson(doc, payload, length);
  if (err) return;

  const char* t = doc["type"] | "";

  if (!strcmp(t, "clear")) {
    clearAllFrames();
    Serial.println("Frames cleared");
    return;
  }

  if (!strcmp(t, "global_enable")) {
    g_globalEnable = (doc["enable"] | 0) != 0;
    Serial.printf("Global TX set to: %d\n", g_globalEnable ? 1 : 0);
    return;
  }

  if (!strcmp(t, "set")) {
    uint32_t id = doc["id"] | 0;
    bool ext = (doc["ext"] | 0) != 0;
    uint8_t dlc = doc["dlc"] | 8;
    uint32_t period = doc["period"] | 100;
    bool enabled = (doc["enabled"] | 0) != 0;

    JsonArray arr = doc["data"].as<JsonArray>();
    if (arr.isNull()) return;

    int idx = allocFrame(id, ext);
    if (idx < 0) return;

    frames[idx].dlc = (dlc > 8) ? 8 : dlc;
    frames[idx].periodMs = max<uint32_t>(1, period);
    frames[idx].enabled = enabled;
    frames[idx].nextDue = millis() + frames[idx].periodMs;

    int i = 0;
    for (JsonVariant v : arr) {
      if (i >= 8) break;
      frames[idx].data[i++] = (uint8_t)(v.as<int>() & 0xFF);
    }
  }
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // MCP2515 init
  SPI.begin(); // default ESP32: SCK=18, MISO=19, MOSI=23
  byte status;
  if (MCP2515_OSC_8MHZ) status = CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ);
  else                 status = CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);

  if (status == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
    Serial.println("CAN init OK");
  } else {
    Serial.println("CAN init FAIL (check bitrate/osc/wiring)");
  }

  pinMode(PIN_INT, INPUT);

  // Wi-Fi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP()); // typically 192.168.4.1

  // HTTP server
  server.on("/", HTTP_GET, handleRoot);
  server.begin();

  // WebSocket server
  ws.begin();
  ws.onEvent(wsEvent);

  Serial.println("HTTP :80, WS :81");
}

void loop() {
  server.handleClient();
  ws.loop();

  // Periodic CAN TX (hard-gated)
  if (!g_globalEnable) return;

  uint32_t now = millis();
  for (int i = 0; i < MAX_FRAMES; i++) {
    if (!frames[i].used || !frames[i].enabled) continue;

    if ((int32_t)(now - frames[i].nextDue) >= 0) {
      if (frames[i].ext) {
        CAN.sendMsgBuf(frames[i].id, 1, frames[i].dlc, frames[i].data);
      } else {
        CAN.sendMsgBuf(frames[i].id, 0, frames[i].dlc, frames[i].data);
      }
      frames[i].nextDue = now + frames[i].periodMs;
    }
  }
}
