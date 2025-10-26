/*
## Lightning Senor V1
Author: Gerald Oberschmidt
Datum: 25.10.2025

Verwendet Board: DFRobot Lightning Sensor V1.0 https://www.dfrobot.com/product-1828.html
mit AS3935 Franklin lightning sensor IC and Coilcraft MA5532-AE
Open Source Alternative zum Board: https://github.com/sparkfun/SparkFun_AS3935_Lightning_Detector
Chip: https://www.sciosense.com/as3935-franklin-lightning-sensor-ic/
Library: https://github.com/sparkfun/SparkFun_AS3935_Lightning_Detector_Arduino_Library



## Verdrahtung (ESP32‑C3 Super Mini, I²C)

- **ESP32‑C3 SDA (GPIO6) → AS3935 SDA**
- **ESP32‑C3 SCL (GPIO7) → AS3935 SCL**
- **3V3 ↔ VCC**, **GND ↔ GND**
- **AS3935 IRQ → ESP32‑C3 GPIO10** (im Code `PIN_AS3935_IRQ`)
- 4 LEDs (mit Vorwiderständen) an die Pins `3, 4, 5, 1` gegen GND

> Manche C3-Boards führen andere Default-I²C-Pins heraus. Wenn dein Board andere Pins nutzt, stelle `PIN_I2C_SDA/SCL` entsprechend um. Vermeide nach Möglichkeit GPIO0/2/8/9 (Boot/Strapping) und Ports, die durch USB‑CDC/UART belegt sind.

---

## Hinweise & Tuning

- **Indoor/Outdoor:** `setIndoor(true)` für drinnen, `false` für draußen (empfohlen bei Outdoor-Montage).
- **Filter:** `setNoiseFloor`, `setSpikeRejection`, `setWatchdogThreshold` an deine Umgebung anpassen, sonst gibt’s viele Fehlalarme.
- **Distanz-Buckets & LEDs:** aktuell: ≤5 km (alle LEDs), 6–10 km (3 LEDs), 11–20 km (2 LEDs), >20 km (1 LED), 63 km (aus). Feinjustiere nach Bedarf.
- **Zeiten:** Die History hält max. 24 h. Über `/api/events?since=86400` bekommst du 24 h.
- **Sicherheit:** `secrets.h` nicht in Repos einchecken. Für produktive Nutzung gern „WiFiManager“ nutzen.
- **Stromversorgung:** AS3935 ist empfindlich – saubere 3V3, kurze Leitungen und ggf. EMV-Filter/Schirmung beachten.

*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SparkFun_AS3935.h> // SparkFun AS3935 Lightning Detector
#include <Wire.h>
#include <vector>
#include <deque>
#include <algorithm>
#include <time.h>

#include "secrets.h"

// =============================
// Pins & Konfiguration
// =============================
// I2C-Pins für ESP32-C3 Super Mini (häufig: SDA=8, SCL=9) – bei Bedarf anpassen
static constexpr int PIN_I2C_SDA = 8;
static constexpr int PIN_I2C_SCL = 9;

// AS3935 INT-Pin (IRQ) – wähle einen freien GPIO (z. B. 10)
static constexpr int PIN_AS3935_IRQ = 10;

// 4 LEDs zur Distanzanzeige (GPIOs an dein Board anpassen)
static constexpr int LED1 = 3;   // sehr nahe (≤5 km)
static constexpr int LED2 = 4;   // nahe (6–10 km)
static constexpr int LED3 = 5;   // mittel (11–20 km)
static constexpr int LED4 = 1;   // weit (>20 km)  (vermeide 0/2/8/9 wegen Boot/Straps, falls möglich)  // weit (>20 km)

// I2C-Adresse des AS3935 (SparkFun Breakout meist 0x03, als 7-bit = 0x03 / Library intern handled)
// Die SparkFun-Lib erwartet die 7-bit Adresse (Default 0x03). Manche Breakouts nutzen 0x02/0x03 (gelötet). Bei Problemen prüfen!
static constexpr uint8_t AS3935_I2C_ADDR = 0x03;

// =============================
// Globale Objekte
// =============================
SparkFun_AS3935 lightning(AS3935_I2C_ADDR);
WebServer server(80);
volatile bool irqFlag = false; // vom ISR gesetzt
static uint32_t tLastPoll = 0;

// =============================
// Datenstrukturen
// =============================
struct LightningEvent {
  time_t ts;        // Unix Timestamp (UTC)
  uint8_t distance; // 1..63 km (63 = out of range laut Datenblatt), 0 = Sturm sehr nahe / über Kopf
  uint32_t energy;  // „Energy“ Rohwert aus dem Sensor (nicht kalibriert)
  uint32_t event;
  bool irq;
};

// Ereignisliste (max ~ 2000 Einträge ≈ ausreichend für 24h bei moderater Aktivität)
static std::deque<LightningEvent> history;
static const size_t HISTORY_MAX = 2000;

// Poll-Status
static uint8_t lastDistance = 63;  // zuletzt gemeldete Distanz (km)
static uint32_t lastEnergy = 0;    // zuletzt gemeldete Energie (keine pyhsikalische Bedeutung)
static time_t lastEventTs = 0;
static uint8_t lastEvent = 0;
static bool AS3935_started = false; // Flag ob der Sensor gesartet ist
static bool AS3935_irq = false;     // Flag der IRQ getriggert wurde

// =============================
// Hilfsfunktionen
// =============================
void IRAM_ATTR onAs3935Interrupt() { irqFlag = true; }

static String tsToIso8601(time_t t) {
  struct tm timeinfo;
  gmtime_r(&t, &timeinfo);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}

static void trimHistoryOlderThan(time_t cutoff) {
  while (!history.empty() && history.front().ts < cutoff) {
    history.pop_front();
  }
}

static void setLedsForDistance(uint8_t km) {
  // km: 0 = very close/overhead, 1..63; 63 == out of range
  bool l1=false, l2=false, l3=false, l4=false;
  switch (km) {
    case 63: 
      break;
    case 40:
      l1=true;
      break;
    case 37:
      l2=true;
      break;
    case 34:
      l1=l2=true;
      break;
    case 31:
      l3=true;
      break;
    case 27:
      l1=l3=true;
      break;
    case 24:
      l2=l3=true;
      break;
    case 20:
      l1=l2=l3=true;
      break;
    case 17:
      l4=true;
      break;
    case 14:
      l1=l4=true;
      break;
    case 12:
      l2=l4=true;
      break;
    case 10:
      l1=l2=l4=true;
      break;
    case  8:
      l3=l4=true;
      break;
    case  6:
      l1=l3=l4=true;
      break;
    case  5:
      l2=l3=l4=true;
      break;
    case  1:
      l1=l2=l3=l4=true;
      break;
  }

  digitalWrite(LED1, l1 ? HIGH : LOW);
  digitalWrite(LED2, l2 ? HIGH : LOW);
  digitalWrite(LED3, l3 ? HIGH : LOW);
  digitalWrite(LED4, l4 ? HIGH : LOW);
}

static unsigned long nextRetryMs = 0;
void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("[WiFi] STA_START → begin connect");
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("[WiFi] Connected to AP (BSSID %s, ch %d)\n",
                    WiFi.BSSIDstr().c_str(), WiFi.channel());
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
      nextRetryMs = 0; // reset backoff
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.printf("[WiFi] Disconnected → retry soon\n");
      // kleiner Backoff (10 s)
      nextRetryMs = millis() + 10000;
      break;
    default: break;
  }
}

static bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);                 // keine NVS-Schreiberei beim Reconnect
  WiFi.setAutoReconnect(true);            // automatische Reconnects erlauben
  esp_wifi_set_ps(WIFI_PS_NONE);          // Stromsparen aus → stabilere Verbindungen
  WiFi.onEvent(onWiFiEvent);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WLAN verbinden...");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Verbunden: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WLAN-Verbindung fehlgeschlagen (Starte AP-Modus wäre Option)");
  }

  return (WiFi.status() == WL_CONNECTED);

}

static void setupTime() {
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov"); // EU Regel
  // etwas warten, bis Zeit gesetzt ist
  for (int i=0;i<10;i++) {
    time_t now = time(nullptr);
    if (now > 1600000000) break; // plausibel
    delay(500);
  }
}

// =============================
// Webserver-Handler
// =============================
static void handleRoot() {
  String html = R"HTML(
<!doctype html>
<html lang="de"><head><meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>AS3935 Lightning Monitor</title>
<style>body{font-family:system-ui,Segoe UI,Roboto,Arial;margin:2rem;max-width:800px} .card{border:1px solid #ddd;border-radius:12px;padding:1rem;margin:1rem 0;box-shadow:0 2px 8px rgba(0,0,0,.05)} code{background:#f5f5f5;padding:.1rem .3rem;border-radius:4px}</style>
</head>
<body>
<h1>AS3935 Lightning Monitor</h1>

<div class="card">
  <h2>LED-Status</h2>
  <div id="led-panel">
    <svg viewBox="0 0 320 90" class="led-svg" aria-label="LED Status">
      <!-- Filter für Glow -->
      <defs>
        <filter id="glow" x="-50%" y="-50%" width="200%" height="200%">
          <feGaussianBlur stdDeviation="3" result="coloredBlur"/>
          <feMerge>
            <feMergeNode in="coloredBlur"/>
            <feMergeNode in="SourceGraphic"/>
          </feMerge>
        </filter>
      </defs>

      <!-- LED1 Grün -->
      <g id="led1" class="led" data-color="green" transform="translate(40,45)">
        <circle r="22" class="ring"/>
        <circle r="18" class="dot"/>
        <text y="36" text-anchor="middle" class="label">LED1</text>
      </g>

      <!-- LED2 Grün -->
      <g id="led2" class="led" data-color="green" transform="translate(120,45)">
        <circle r="22" class="ring"/>
        <circle r="18" class="dot"/>
        <text y="36" text-anchor="middle" class="label">LED2</text>
      </g>

      <!-- LED3 Gelb -->
      <g id="led3" class="led" data-color="yellow" transform="translate(200,45)">
        <circle r="22" class="ring"/>
        <circle r="18" class="dot"/>
        <text y="36" text-anchor="middle" class="label">LED3</text>
      </g>

      <!-- LED4 Rot -->
      <g id="led4" class="led" data-color="red" transform="translate(280,45)">
        <circle r="22" class="ring"/>
        <circle r="18" class="dot"/>
        <text y="36" text-anchor="middle" class="label">LED4</text>
      </g>
    </svg>
    <div class="legend">
      <span class="pill g">LED1</span>
      <span class="pill g">LED2</span>
      <span class="pill y">LED3</span>
      <span class="pill r">LED4</span>
    </div>
  </div>
</div>

<style>
  .led-svg { width:100%; height:auto; max-height:140px; }
  .led .ring { fill:#f3f3f3; stroke:#d0d0d0; stroke-width:2; }
  .led .dot  { fill:#bfbfbf; }

  .led.on[data-color="green"]  .dot { fill:#20c064; filter:url(#glow); }
  .led.on[data-color="yellow"] .dot { fill:#f5c542; filter:url(#glow); }
  .led.on[data-color="red"]    .dot { fill:#ff5555; filter:url(#glow); }
  .led .label { font: 12px/1.2 system-ui, Segoe UI, Roboto, Arial; fill:#555; }

  .legend { margin-top:.6rem; display:flex; gap:.5rem; flex-wrap:wrap; }
  .pill { font:12px system-ui, Segoe UI, Roboto, Arial; padding:.2rem .5rem; border-radius:999px; border:1px solid #ddd; background:#fafafa; }
  .pill.g { border-color:#bfe8cf; background:#ebfff4; }
  .pill.y { border-color:#f1e2a9; background:#fff8e1; }
  .pill.r { border-color:#f2b5b5; background:#fff0f0; }
</style>



<div class="card" id="live">Lädt Live-Daten…</div>


<style>
  body{font-family:system-ui,Segoe UI,Roboto,Arial;margin:20px;max-width:980px}
  .row{display:grid;gap:16px}
  .card{border:1px solid #ddd;border-radius:12px;padding:16px;box-shadow:0 2px 8px rgba(0,0,0,.06)}
  h2{margin:.2rem 0 1rem}
  canvas{width:100%;height:260px;border-radius:8px;background:#fff}
  .meta{font-size:.9rem;color:#555}
</style>
</head>
<body>
  
  <div class="row">
    <div class="card">
      <h2>Distanz (km) – letzte 60 Minuten</h2>
      <canvas id="dist"></canvas>
    </div>
    <div class="card">
      <h2>log10(Energie) – letzte 60 Minuten</h2>
      <canvas id="energy"></canvas>
    </div>
  </div>

  <div class="card" id="list">Lädt Events…</div>

  <div class="card">
    <h2>API</h2>
    <ul>
      <li><code>/api/events?since=3600</code> – Ereignisse letzte Stunde</li>
      <li><code>/api/live</code> – Status</li>
      <li><code>/api/stats?range=hour|day</code> – Statistik</li>
    </ul>
  </div>

<script>
const MINUTES = 60;
const PAD = {l:48, r:12, t:12, b:28};

function pxMap(x, x0, x1, w) {
  return PAD.l + (x - x0) * (w - PAD.l - PAD.r) / (x1 - x0);
}
function pyMap(y, y0, y1, h) {
  return (h - PAD.b) - (y - y0) * (h - PAD.t - PAD.b) / (y1 - y0);
}
function drawAxes(ctx, w, h, x0, x1, y0, y1, xLabel, yLabel) {
  ctx.clearRect(0,0,w,h);
  ctx.lineWidth = 1; ctx.strokeStyle = "#888"; ctx.fillStyle="#000";
  // Axen
  ctx.beginPath();
  ctx.moveTo(PAD.l, PAD.t); ctx.lineTo(PAD.l, h-PAD.b); ctx.lineTo(w-PAD.r, h-PAD.b);
  ctx.stroke();
  ctx.font = "12px system-ui,Segoe UI,Arial";

  // X-Ticks (alle 10 Minuten)
  for (let m = x0; m <= x1; m+=10) {
    const x = pxMap(m, x0, x1, w);
    ctx.strokeStyle="#ccc";
    ctx.beginPath(); ctx.moveTo(x, h-PAD.b); ctx.lineTo(x, PAD.t); ctx.stroke();
    ctx.fillStyle="#333";
    ctx.fillText(String(m), x-8, h-8);
  }
  // Y-Ticks (5 Schritte)
  for (let i=0;i<=5;i++){
    const yv = y0 + (i*(y1-y0)/5);
    const y = pyMap(yv, y0, y1, h);
    ctx.strokeStyle="#eee";
    ctx.beginPath(); ctx.moveTo(PAD.l, y); ctx.lineTo(w-PAD.r, y); ctx.stroke();
    ctx.fillStyle="#333";
    ctx.fillText(yv.toFixed( (y1-y0)>20 ? 0 : 1 ), 8, y+4);
  }
  // Labels
  ctx.fillStyle="#555";
  ctx.fillText(xLabel, w/2-40, h-4);
  ctx.save(); ctx.translate(14, h/2); ctx.rotate(-Math.PI/2); ctx.fillText(yLabel, -40, 0); ctx.restore();
}

function scatter(ctx, w, h, points, x0, x1, y0, y1) {
  ctx.fillStyle = "#0a84ff"; // Distanz / Energie Punkte
  for (const p of points) {
    const x = pxMap(p.x, x0, x1, w);
    const y = pyMap(p.y, y0, y1, h);
    ctx.beginPath(); ctx.arc(x, y, 3, 0, Math.PI*2); ctx.fill();
  }
}

function setLedState(id, on) {
  const el = document.getElementById(id);
  if (!el) return;
  el.classList.toggle('on', !!on);
}

async function last_events() {
  const live = await fetch('/api/live').then(r=>r.json());
  document.getElementById('live').innerHTML = 
    `<b>Sensor gestartet    :</b> ${live.started}` +
    `<br><b>Letzte Distanz  :</b> ${live.last_distance_km} km` +
    `<br><b>Letzte Energie  :</b> ${live.last_energy}` +
    `<br><b>Letztes Ereignis:</b> ${live.last_event_string} um ${live.last_event_iso || '—'} durch ${live.last_event_trigger}` +
    `<br><b>Uptime (s)      :</b> ${live.uptime_s}`;
  const evts = await fetch('/api/events?since=3600').then(r=>r.json());
  document.getElementById('list').innerHTML = `<b>${evts.events.length}</b> Ereignisse (letzte Stunde)`+
    `<pre>${JSON.stringify(evts, null, 2)}</pre>`;

  // LED-Status setzen (du hast doc["l1"]..doc["l4"])
  setLedState('led1', live.l1);
  setLedState('led2', live.l2);
  setLedState('led3', live.l3);
  setLedState('led4', live.l4);
}

async function load() {
  const nowSec = Math.floor(Date.now()/1000);

  // Events der letzten Stunde
  const ev = await fetch('/api/events?since=3600').then(r=>r.json()).catch(_=>({events:[]}));
  const events = (ev.events||[]).map(e=>{
    const minsAgo = Math.max(0, Math.round((nowSec - (e.ts||nowSec))/60));
    const dist = (typeof e.distance_km === 'number') ? -e.distance_km : -63;
    const energy = Math.max(0, Number(e.energy||0));
    const elog = Math.log10(energy+1); // gegen 0 stabil
    return {minsAgo, dist, elog};
  }).filter(e=>e.minsAgo<=MINUTES);

  // Sortiere nach Minuten (aufsteigend 0..60 für schöne Linien)
  events.sort((a,b)=>a.minsAgo-b.minsAgo);

  // --- Distanz-Chart ---
  const c1 = document.getElementById('dist');
  c1.width = c1.clientWidth * window.devicePixelRatio;
  c1.height = c1.clientHeight * window.devicePixelRatio;
  const g1 = c1.getContext('2d'); g1.scale(window.devicePixelRatio, window.devicePixelRatio);

  const x0 = 0, x1 = MINUTES;
  const y0d = -63, y1d = 0; // 63 = out of range
  drawAxes(g1, c1.clientWidth, c1.clientHeight, x0, x1, y0d, y1d, "Minuten ago", "km");
  scatter(g1, c1.clientWidth, c1.clientHeight, events.map(e=>({x:e.minsAgo,y:e.dist})), x0, x1, y0d, y1d);

  // --- Energie-Chart ---
  const c2 = document.getElementById('energy');
  c2.width = c2.clientWidth * window.devicePixelRatio;
  c2.height = c2.clientHeight * window.devicePixelRatio;
  const g2 = c2.getContext('2d'); g2.scale(window.devicePixelRatio, window.devicePixelRatio);

  // Y-Range für log10(energy): automatisch aus Daten (Fallback 0..6)
  let ymin = 0, ymax = 6;
  if (events.length) {
    ymin = 0; // wir starten bei 0
    ymax = Math.max(1, Math.ceil(Math.max(...events.map(e=>e.elog))*1.1));
    ymax = Math.min(8, ymax); // Deckel drauf
  }
  drawAxes(g2, c2.clientWidth, c2.clientHeight, x0, x1, ymin, ymax, "Minuten ago", "log10(E)");
  scatter(g2, c2.clientWidth, c2.clientHeight, events.map(e=>({x:e.minsAgo,y:e.elog})), x0, x1, ymin, ymax);
}

load();
last_events();

setInterval(last_events, 10000);
setTimeout(() => setInterval(load, 10000), 2000); // 2 s versetzt

</script>


</body></html>
)HTML";
  server.send(200, "text/html", html);
}

static void handleLive() {
  DynamicJsonDocument doc(1024);
  doc["ip"] = WiFi.localIP().toString();
  doc["last_distance_km"] = lastDistance;
  doc["last_energy"] = lastEnergy;;
  doc["last_event_ts"] = (int64_t)lastEventTs;
  // 0 = keine, 1 = Noise, 4 = Disturber, 8 = Lightning (abhängig von Lib – Doku prüfen)
  String lastEventString;
  if (lastEvent==0){
    lastEventString=String("Kein");
  } else if (lastEvent == 1) {
    lastEventString=String("Rauschen");
  } else if (lastEvent == 4) {
    lastEventString=String("Störer");
  } else if (lastEvent == 8) {
    lastEventString=String("Blitz");
  } 
  lastEventString=lastEventString + (" :: Wert binär =") + String(lastEvent,BIN);
  doc["last_event_string"] = lastEventString;
  doc["last_event_iso"] = lastEventTs ? tsToIso8601(lastEventTs) : String("");
  doc["last_event_trigger"] = AS3935_irq ? String("Interrupt") : String("Polling");
  doc["uptime_s"] = (uint32_t)(millis()/1000);
  doc["started"] = AS3935_started ? String("Ja, I2C Up") : String("Nein, I2C Down");

  // LED-Status in JSON exportieren (HIGH = an)
  // Falls deine Logik invertiert ist, entsprechend anpassen.
  doc["l1"] = (digitalRead(LED1) == HIGH);
  doc["l2"] = (digitalRead(LED2) == HIGH);
  doc["l3"] = (digitalRead(LED3) == HIGH);
  doc["l4"] = (digitalRead(LED4) == HIGH);

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

static void handleEvents() {
  // Parameter since=Sekunden (default 3600)
  long sinceSec = 3600;
  if (server.hasArg("since")) {
    sinceSec = server.arg("since").toInt();
    if (sinceSec <= 0) sinceSec = 3600;
  }
  time_t now = time(nullptr);
  time_t cutoff = now - sinceSec;

  DynamicJsonDocument doc(32768);
  JsonArray arr = doc.createNestedArray("events");

  for (auto it = history.rbegin(); it != history.rend(); ++it) {
    if (it->ts < cutoff) break;
    JsonObject o = arr.createNestedObject();
    o["ts"] = (int64_t)it->ts;
    o["iso"] = tsToIso8601(it->ts);
    o["distance_km"] = it->distance;
    o["energy"] = it->energy;
    o["event"] = it->event;
    o["irq"] = it->irq;
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

static void handleStats() {
  String range = server.arg("range");
  long sinceSec = (range == "day") ? 24*3600 : 3600;
  time_t now = time(nullptr);
  time_t cutoff = now - sinceSec;

  uint32_t cnt = 0;
  uint32_t near=0, mid=0, far_=0, oor=0;

  for (const auto &e : history) {
    if (e.ts < cutoff) continue;
    cnt++;
    if (e.distance == 63) oor++;
    else if (e.distance <= 5) near++;
    else if (e.distance <= 15) mid++;
    else far_++;
  }

  DynamicJsonDocument doc(1024);
  doc["range_s"] = sinceSec;
  doc["count"] = cnt;
  JsonObject b = doc.createNestedObject("buckets");
  b["near_<=5km"] = near;
  b["mid_6-15km"] = mid;
  b[">15km"] = far_;
  b["out_of_range"] = oor;

  String out; serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// =============================
// Setup Sensor
// =============================
static bool initAS3935() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  delay(50);
  //if (!lightning.begin(AS3935_I2C_ADDR, PIN_AS3935_IRQ)) {
  if (!lightning.begin(Wire)) {
    Serial.println("AS3935 nicht gefunden – prüfe Adresse/Verkabelung");
    return false;
  } else {
    AS3935_started = true;
  }

  // Grund-Setup
  lightning.wakeUp();
  lightning.setIndoorOutdoor(true);       // Indoor-Modus (weniger Rauschen). Outdoor: setIndoor(false)
  lightning.maskDisturber(false);  // Störer melden? false = nicht maskieren → Bibliothek meldet Disturber separat

  // Empfindlichkeit/Filter (Werte an deine Umgebung anpassen)
  lightning.setNoiseLevel(2);      // 0..7 (höher = weniger empfindlich ggü. Rauschen)
  lightning.spikeRejection(2);  // 0..7 (höher = robust, evtl. weniger empfindlich)
  lightning.watchdogThreshold(2);// 0..10

  // Clear event registers
  lightning.clearStatistics(true);

  pinMode(PIN_AS3935_IRQ, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_AS3935_IRQ), onAs3935Interrupt, RISING);

  Serial.println("AS3935 initialisiert.");
  return true;
}

// ESP32 C3 Super Mini on-board LED (works with inverted logic)
const int ledPin = 8; 

void shortBlink(int num, int onMs = 200, int offMs = 200) {
  
    for (int i = 0; i < num; ++i){
        digitalWrite(ledPin, LOW);
        delay(onMs);
        digitalWrite(ledPin, HIGH);
        delay(offMs);
    }
}

// =============================
// Setup & Loop
// =============================
void setup() {

  // Blinken zurm Start
  pinMode(ledPin, OUTPUT);
  shortBlink(2);
  
  tLastPoll = 0;

  Serial.begin(115200);
  delay(200);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  setLedsForDistance(63);

  delay(1000);
  bool conwifi=connectWiFi();
  
  if (conwifi){
    shortBlink(5);
  } else {
    shortBlink(2,1000,1000);
  }

  setupTime();

  if (!initAS3935()) {
    // weiterlaufen, Webserver hilft beim Debuggen
  }

  // Webserver
  server.on("/", handleRoot);
  server.on("/api/live", handleLive);
  server.on("/api/events", handleEvents);
  server.on("/api/stats", handleStats);
  server.begin();
  Serial.println("HTTP-Server gestartet auf Port 80");
}

void loop() {
  server.handleClient();

  // Alte Einträge entfernen (alle Schleifen-Durchläufe leichte Pflege)
  time_t now = time(nullptr);
  trimHistoryOlderThan(now - 24*3600); // max 24h halten

  AS3935_irq = irqFlag;

  if (irqFlag) {
    noInterrupts();
    irqFlag = false;
    interrupts();

    // Quelle des Interrupts ermitteln
    int intSrc = lightning.readInterruptReg();
    // 0 = keine, 1 = Noise, 4 = Disturber, 8 = Lightning (abhängig von Lib – Doku prüfen)

    lastEvent = intSrc;

    //if (intSrc & 0x08) { // Lightning
    if (intSrc != 0x00) { // Debugging
      uint8_t dist = lightning.distanceToStorm(); // 1..63 km, 0 = sehr nahe, 63 = out of range
      uint32_t energy = lightning.lightningEnergy();
      lastDistance = dist;
      lastEnergy = energy;
      lastEventTs = now;
      setLedsForDistance(dist);

      // in History aufnehmen
      if (history.size() >= HISTORY_MAX) history.pop_front();
      history.push_back({now, dist, energy, lastEvent, AS3935_irq});

      Serial.printf("⚡ Blitz erkannt: Distanz %u km, Energy %lu\n", dist, (unsigned long)energy);

    } else if (intSrc & 0x01) { // Rauschen
      Serial.println("~ Noise detected");
    } else if (intSrc & 0x04) { // Störer
      Serial.println("! Disturber detected");
    } else {
      // Unbekannt / kein Event
    }
  }

  // Optional: alle 10 s Distanz neu abfragen und LEDs aktualisieren
  if (millis() - tLastPoll > 10000) {
    
    if (lastEvent>0){
      Serial.println("regular data polling (Event)");
      tLastPoll = millis();
      uint8_t dist = lightning.distanceToStorm();
      uint32_t energy = lightning.lightningEnergy();
      lastDistance = dist;
      lastEnergy = energy;
      setLedsForDistance(dist);

      // in History aufnehmen     
      if (history.size() >= HISTORY_MAX) history.pop_front();
      history.push_back({now, dist, energy, lastEvent, AS3935_irq});
      lastEvent = 0 ;
    }

  }
}

