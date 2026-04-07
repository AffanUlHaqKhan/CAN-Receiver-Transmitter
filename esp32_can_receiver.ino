#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

#include <SPI.h>
#include <mcp_can.h>

// =================== USER CONFIG ===================
static const char* AP_SSID = "ESP32-CAN-RECEIVER";
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

// ---------------- Web UI HTML ----------------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>ESP32 CAN Receiver</title>
  <style>
    body{font-family:system-ui,sans-serif;margin:16px;max-width:1100px}
    .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
    .pill{padding:2px 10px;border-radius:999px;background:#eee;font-size:12px}
    .ok{color:#166534}.bad{color:#b91c1c}
    table{border-collapse:collapse;width:100%;margin-top:10px}
    th,td{border:1px solid #ddd;padding:6px;font-size:13px;vertical-align:top}
    th{background:#f6f6f6;text-align:left}
    .small{font-size:12px;color:#555}
    .warn{color:#b45309}
    code{background:#f6f6f6;padding:2px 5px;border-radius:6px}
    button{padding:6px 10px}
    input[type=number]{width:120px}
    .mono{font-family:ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;}
  </style>
</head>
<body>
  <h2>ESP32 CAN Receiver</h2>

  <div class="row">
    <span class="pill" id="wsState">WS: connecting…</span>
    <button id="btnClear">Clear table</button>
  </div>

  <p class="small">
    Connect to Wi-Fi <b>ESP32-CAN-RECEIVER</b> → open <code>http://192.168.4.1</code><br/>
    WebSocket: <code>ws://192.168.4.1:81/</code><br/>
    Optional: load DBC to decode Intel/little-endian (@1) signals. Big-endian (@0) skipped.
  </p>

  <div class="row">
    <input id="dbcFile" type="file" accept=".dbc"/>
    <button id="btnLoad">Load DBC for decode</button>
  </div>

  <div id="note" class="small warn"></div>

  <h3>Raw frames (latest first)</h3>
  <table id="rawTbl">
    <tr><th>Time</th><th>ID</th><th>DLC</th><th>Data</th></tr>
  </table>

  <h3>Decoded (if DBC loaded)</h3>
  <table id="decTbl">
    <tr><th>Message</th><th>Signal</th><th>Value</th><th>Unit</th></tr>
  </table>

<script>
let ws;
let db = { messages: [] };
let dbcLoaded = false;

function setWs(text, ok){
  const el = document.getElementById("wsState");
  el.textContent = text;
  el.className = "pill " + (ok ? "ok" : "bad");
}

function wsConnect(){
  ws = new WebSocket(`ws://192.168.4.1:81/`);
  ws.onopen  = () => setWs("WS: connected", true);
  ws.onclose = () => { setWs("WS: disconnected", false); setTimeout(wsConnect, 2000); };
  ws.onerror = () => setWs("WS: error", false);

  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "frame") {
      addRaw(msg);
      if (dbcLoaded) decodeAndShow(msg);
    }
  };
}

function addRaw(f){
  const tbl = document.getElementById("rawTbl");
  const tr = document.createElement("tr");
  const dataHex = f.data.map(b => b.toString(16).padStart(2,"0")).join(" ");
  tr.innerHTML = `<td class="mono">${new Date().toISOString().slice(11,23)}</td>
                  <td class="mono">0x${f.id.toString(16)}</td>
                  <td>${f.dlc}</td>
                  <td class="mono">${dataHex}</td>`;
  // insert after header row
  tbl.insertBefore(tr, tbl.rows[1] || null);

  // keep table small
  while (tbl.rows.length > 51) tbl.deleteRow(tbl.rows.length - 1);
}

// ---------- DBC parsing (tolerant) ----------
function parseDBC(text){
  const lines = text.split(/\\r?\\n/);
  const messages = [];
  let current = null;

  const reBO = /^BO_\\s+(\\d+)\\s+([^:]+)\\s*:\\s*(\\d+)/;
  const reSG = /^\\s*SG_\\s+([^\\s]+)\\s*(?:m\\d+)?\\s*:\\s*(\\d+)\\|(\\d+)@(\\d)([+-])\\s+\\(([^,]+),([^)]+)\\)\\s*(?:\\[\\s*([^|]+)\\|\\s*([^\\]]+)\\s*\\])?\\s*\\"([^\\"]*)\\"/;

  for(const line of lines){
    const mBO = line.match(reBO);
    if (mBO){
      current = { id: parseInt(mBO[1],10), name: mBO[2].trim(), dlc: parseInt(mBO[3],10), signals: [] };
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
        min, max,
        unit: (mSG[10] || "")
      });
    }
  }
  return { messages };
}

// ---------- Decode Intel/little-endian ----------
function getBitsLE(bytes, startBit, bitLen){
  let raw = 0n;
  for(let i=0;i<bitLen;i++){
    const bitPos = startBit + i;
    const byteIndex = Math.floor(bitPos / 8);
    const bitIndex  = bitPos % 8;
    if(byteIndex < 0 || byteIndex >= bytes.length) continue;
    const bitVal = (bytes[byteIndex] >> bitIndex) & 1;
    raw |= (BigInt(bitVal) << BigInt(i));
  }
  return raw;
}

function rawToPhys(sig, rawU){
  // rawU is BigInt unsigned
  let rawNum;
  if(sig.signed){
    // two's complement
    const signBit = 1n << BigInt(sig.bitLen - 1);
    let signedVal = rawU;
    if (rawU & signBit) {
      signedVal = rawU - (1n << BigInt(sig.bitLen));
    }
    rawNum = Number(signedVal);
  } else {
    rawNum = Number(rawU);
  }
  return rawNum * sig.factor + sig.offset;
}

function decodeAndShow(f){
  const bytes = f.data.slice(0, f.dlc); // array numbers
  const msg = db.messages.find(m => m.id === f.id);
  if(!msg) return;

  // warn if msg contains big endian
  // (we just skip those signals)
  const tbl = document.getElementById("decTbl");

  for(const sig of msg.signals){
    if(sig.endian !== "little") continue;

    const rawU = getBitsLE(bytes, sig.startBit, sig.bitLen);
    const phys = rawToPhys(sig, rawU);

    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${msg.name}</td>
                    <td>${sig.name}</td>
                    <td class="mono">${Number.isFinite(phys) ? phys.toFixed(3) : phys}</td>
                    <td>${sig.unit||""}</td>`;
    tbl.insertBefore(tr, tbl.rows[1] || null);
  }

  while (tbl.rows.length > 101) tbl.deleteRow(tbl.rows.length - 1);
}

// UI buttons
document.getElementById("btnClear").onclick = () => {
  const raw = document.getElementById("rawTbl");
  const dec = document.getElementById("decTbl");
  while (raw.rows.length > 1) raw.deleteRow(1);
  while (dec.rows.length > 1) dec.deleteRow(1);
};

document.getElementById("btnLoad").onclick = async () => {
  const f = document.getElementById("dbcFile").files[0];
  if(!f) return;
  const text = await f.text();
  db = parseDBC(text);
  dbcLoaded = true;

  let hasBE = false;
  for(const m of db.messages) for(const s of m.signals) if(s.endian !== "little") hasBE = true;
  document.getElementById("note").textContent = hasBE ? "Warning: Big-endian (@0) signals exist; this decoder skips them." : "";
};

wsConnect();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void wsEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    Serial.printf("WS client %u connected\n", clientNum);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("WS client %u disconnected\n", clientNum);
  }
}

// Send a CAN frame to all WS clients as JSON
void broadcastFrame(uint32_t id, bool ext, uint8_t dlc, const uint8_t* data) {
  StaticJsonDocument<256> doc;
  doc["type"] = "frame";
  doc["id"] = id;
  doc["ext"] = ext ? 1 : 0;
  doc["dlc"] = dlc;
  JsonArray arr = doc.createNestedArray("data");
  for (int i = 0; i < dlc; i++) arr.add(data[i]);

  char out[256];
  size_t n = serializeJson(doc, out, sizeof(out));
  ws.broadcastTXT(out, n);
}

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

  // HTTP
  server.on("/", HTTP_GET, handleRoot);
  server.begin();

  // WS
  ws.begin();
  ws.onEvent(wsEvent);

  Serial.println("HTTP :80, WS :81");
}

void loop() {
  server.handleClient();
  ws.loop();

  // MCP2515 interrupt goes LOW when message received
  if (digitalRead(PIN_INT) == LOW) {
    unsigned long rxId = 0;
    byte len = 0;
    byte buf[8];

    if (CAN.readMsgBuf(&rxId, &len, buf) == CAN_OK) {
      bool ext = (rxId > 0x7FF); // heuristic
      // Serial raw
      Serial.print("RX 0x");
      Serial.print((uint32_t)rxId, HEX);
      Serial.print(" DLC=");
      Serial.print((int)len);
      Serial.print(" Data=");
      for (int i = 0; i < len; i++) {
        if (buf[i] < 16) Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      // Web broadcast
      broadcastFrame((uint32_t)rxId, ext, len, buf);
    }
  }

  yield();
}
