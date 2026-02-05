#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <math.h>

// ---------------------- USER SETTINGS ----------------------

// Hall sensor digital output (D0 from module)
#define HALL_PIN        2      // Change if you wired D0 to a different pin

// GPS UART pins (ESP32-C3)
// These are the pins you said you use for GPS
#define GPS_RX_PIN      20     // ESP32-C3 pin receiving data from GPS TX
#define GPS_TX_PIN      21     // ESP32-C3 pin sending data to GPS RX (usually unused)

// Wi-Fi AP settings (ESP32 runs as Access Point)
const char* AP_SSID = "BikeMonitor";
const char* AP_PASS = "12345678";  // minimum 8 chars

// Home / fixed reference point (for distance AND for fake SMS location)
const double HOME_LAT = 14.599022545110033;
const double HOME_LON = 121.00125564336915;

// Fixed "fake" location for SMS
const char* FIXED_LOCATION_NAME = "Home";
const char* FIXED_MAP_URL =
  "https://www.google.com/maps?q=14.599022545110033,121.00125564336915";

// Wheel parameters
const float WHEEL_CIRCUMFERENCE_M = 2.1;   // meters

// Speed calculation interval
const unsigned long SPEED_INTERVAL_MS = 1000;  // calculate speed every 1 second

// Speed limit for overspeed indicator
const float SPEED_LIMIT_KMH = 15.0;      // <<< change this to your preferred limit

// SMS intervals
const unsigned long STATUS_SMS_INTERVAL_MS = 60000;  // 60s periodic status SMS

// ---------------------- GLOBALS ----------------------------

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);      // UART1 for GPS (change to 0 if your board needs that)

// Web server
WebServer server(80);

// Hall / speed
volatile unsigned long pulseCount = 0;      // wheel pulses from Hall sensor
unsigned long lastSpeedCalcTime = 0;
float currentSpeedKmh = 0.0;

// SMS via phone-gateway state
String parentPhone = "+639159341875";  // can be edited in web UI
String gatewayHost = "";               // phone IP on ESP32 AP (set in web UI)
uint16_t gatewayPort = 0;              // phone SMS app port (set in web UI)

bool overspeedAlertSent = false;
unsigned long lastStatusSmsTime = 0;
bool startupSmsSent = false;

// ---------------------- MATH HELPERS -----------------------

double deg2rad(double deg) {
  return deg * 3.141592653589793 / 180.0;
}

// Haversine distance in meters between two GPS points
double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Earth radius in meters
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat/2) * sin(dLat/2) +
             cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
             sin(dLon/2) * sin(dLon/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

// ---------------------- ISR -------------------------------

void IRAM_ATTR hallISR() {
  pulseCount++;
}

// ---------------------- SPEED CALC ------------------------

void updateSpeed() {
  unsigned long now = millis();
  if (now - lastSpeedCalcTime >= SPEED_INTERVAL_MS) {
    lastSpeedCalcTime = now;

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    if (pulses == 0) {
      currentSpeedKmh = 0.0;
    } else {
      float intervalSec = SPEED_INTERVAL_MS / 1000.0;
      float revPerSec = pulses / intervalSec;
      float speedMps = revPerSec * WHEEL_CIRCUMFERENCE_M;
      currentSpeedKmh = speedMps * 3.6;
    }
  }
}

// ---------------------- GPS HANDLING ----------------------

void readGPS() {
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    gps.encode(c);
  }
}

// Distance from fixed home using real GPS (for UI only)
double currentDistanceFromHome() {
  if (!gps.location.isValid()) return 0.0;
  return distanceMeters(HOME_LAT, HOME_LON, gps.location.lat(), gps.location.lng());
}

// ---------------------- JSON ESCAPE ------------------------

// Escape JSON special chars so the SMS body is valid JSON
String jsonEscape(const String &s) {
  String out;
  out.reserve(s.length() + 10);
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    if (c == '\\') out += "\\\\";
    else if (c == '\"') out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else out += c;
  }
  return out;
}

// ---------------------- SMS VIA PHONE GATEWAY --------------
//
// Phone runs an HTTP server at http://<gatewayHost>:<gatewayPort>/send-sms
// Body: {"phone":"<number>","message":"<text>"}

bool sendSmsViaGateway(const String &to, const String &message) {
  if (gatewayHost.length() == 0 || gatewayPort == 0) {
    Serial.println("[SMS] Gateway host/port not set.");
    return false;
  }
  if (to.length() < 5) {
    Serial.println("[SMS] Invalid phone number.");
    return false;
  }

  String url = "http://" + gatewayHost + ":" + String(gatewayPort) + "/send-sms";
  Serial.print("[SMS] POST ");
  Serial.println(url);

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String body = "{";
  body += "\"phone\":\"" + jsonEscape(to) + "\",";
  body += "\"message\":\"" + jsonEscape(message) + "\"";
  body += "}";

  Serial.println("[SMS] JSON body:");
  Serial.println(body);

  int code = http.POST(body);
  Serial.print("[SMS] HTTP status: ");
  Serial.println(code);

  String resp = http.getString();
  Serial.println("[SMS] Response:");
  Serial.println(resp);

  http.end();
  return (code > 0 && code < 400);
}

// ---------------------- SMS MESSAGE BUILDERS ---------------
String buildGpsLocationStringForSms() {
  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    String s = "Location: Lat ";
    s += String(lat, 6);
    s += ", Lon ";
    s += String(lon, 6);
    return s;
  } else {
    return "Location: GPS not available";
  }
}

String buildGpsMapsUrlForSms() {
  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    String url = "https://www.google.com/maps?q=";
    url += String(lat, 6);
    url += ",";
    url += String(lon, 6);
    return url;
  } else {
    // if you want zero fallback, return empty string:
    return "";

    // OR if you still want a backup fixed point, you could do:
    // return String(FIXED_MAP_URL);
  }
}

String buildOverspeedMessage(bool overspeedNow) {
  String msg = "OVERSPEED ALERT\n";
  msg += "Speed: ";
  msg += String(currentSpeedKmh, 1);
  msg += " km/h (limit ";
  msg += String(SPEED_LIMIT_KMH, 1);
  msg += ")\n";

  // Distance from home (only if GPS has a fix)
  double distM = currentDistanceFromHome();
  if (gps.location.isValid() && distM > 0) {
    msg += "Distance from home: ";
    if (distM < 1000) {
      msg += String(distM, 0);
      msg += " m\n";
    } else {
      msg += String(distM / 1000.0, 2);
      msg += " km\n";
    }
  }

  // Live GPS location text
  msg += buildGpsLocationStringForSms();
  msg += "\n";

  // Live Google Maps link (if available)
  String mapsUrl = buildGpsMapsUrlForSms();
  if (mapsUrl.length() > 0) {
    msg += "Google Maps: ";
    msg += mapsUrl;
  }

  return msg;
}


String buildStatusMessage(bool overspeedNow) {
  String msg = "Bike status\n";
  msg += "Speed: ";
  msg += String(currentSpeedKmh, 1);
  msg += " km/h\n";

  msg += "Overspeeding: ";
  msg += overspeedNow ? "YES\n" : "NO\n";

  // Distance from home (only if GPS has a fix)
  double distM = currentDistanceFromHome();
  if (gps.location.isValid() && distM > 0) {
    msg += "Distance from home: ";
    if (distM < 1000) {
      msg += String(distM, 0);
      msg += " m\n";
    } else {
      msg += String(distM / 1000.0, 2);
      msg += " km\n";
    }
  }

  // Live GPS location text
  msg += buildGpsLocationStringForSms();
  msg += "\n";

  // Live Google Maps link (if available)
  String mapsUrl = buildGpsMapsUrlForSms();
  if (mapsUrl.length() > 0) {
    msg += "Google Maps: ";
    msg += mapsUrl;
  }

  return msg;
}


void maybeSendStartupSms() {
  if (startupSmsSent) return;
  if (millis() < 10000) return; // wait a bit after boot
  if (parentPhone.length() < 5) return;
  if (gatewayHost.length() == 0 || gatewayPort == 0) return;

  String msg = "Smart Bike is online.\n";
  msg += "Web UI: http://";
  msg += WiFi.softAPIP().toString();
  msg += "\n";

  // Add GPS if available
  msg += buildGpsLocationStringForSms();
  msg += "\n";

  String mapsUrl = buildGpsMapsUrlForSms();
  if (mapsUrl.length() > 0) {
    msg += "Google Maps: ";
    msg += mapsUrl;
  }

  Serial.println("[STARTUP] Sending startup SMS via gateway...");
  bool ok = sendSmsViaGateway(parentPhone, msg);
  Serial.print("[STARTUP] sendSmsViaGateway() result: ");
  Serial.println(ok ? "SUCCESS" : "FAIL");

  startupSmsSent = true;
}


// overspeed + periodic status logic
void handleSmsLogic() {
  if (parentPhone.length() < 5 || gatewayHost.length() == 0 || gatewayPort == 0) return;

  unsigned long now = millis();
  bool overspeedNow = currentSpeedKmh > SPEED_LIMIT_KMH;

  // ---- OVERSPEED ALERT (once per event) ----
  if (overspeedNow && !overspeedAlertSent) {
    String msg = buildOverspeedMessage(overspeedNow);
    Serial.println("[ALERT] Sending overspeed SMS via gateway...");
    bool ok = sendSmsViaGateway(parentPhone, msg);
    Serial.print("[ALERT] sendSmsViaGateway() result: ");
    Serial.println(ok ? "SUCCESS" : "FAIL");
    overspeedAlertSent = true;
  }

  // reset overspeed flag with hysteresis
  if (!overspeedNow && currentSpeedKmh < SPEED_LIMIT_KMH - 2.0) {
    overspeedAlertSent = false;
  }

  // ---- PERIODIC STATUS SMS ----
  if (now - lastStatusSmsTime >= STATUS_SMS_INTERVAL_MS) {
    lastStatusSmsTime = now;

    String msg = buildStatusMessage(overspeedNow);
    Serial.println("[STATUS] Sending periodic status SMS via gateway...");
    bool ok = sendSmsViaGateway(parentPhone, msg);
    Serial.print("[STATUS] sendSmsViaGateway() result: ");
    Serial.println(ok ? "SUCCESS" : "FAIL");
  }
}

// ---------------------- WEB HANDLERS ----------------------

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Smart Bike Monitor</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      padding: 0;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Arial, sans-serif;
      background: radial-gradient(circle at top, #1f2937 0, #020617 55%);
      color: #f9fafb;
    }
    .wrapper {
      max-width: 480px;
      margin: 0 auto;
      padding: 16px;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      gap: 12px;
    }
    .header {
      text-align: center;
      margin-bottom: 4px;
    }
    .title {
      font-size: 1.7rem;
      font-weight: 700;
      letter-spacing: 0.03em;
      margin-bottom: 4px;
    }
    .subtitle {
      font-size: 0.9rem;
      color: #9ca3af;
    }
    .chip-row {
      display: flex;
      justify-content: center;
      gap: 6px;
      margin-top: 6px;
      flex-wrap: wrap;
    }
    .chip {
      font-size: 0.7rem;
      padding: 2px 8px;
      border-radius: 999px;
      background: rgba(15,23,42,0.9);
      border: 1px solid rgba(148,163,184,0.3);
      color: #e5e7eb;
    }
    .card {
      background: rgba(15, 23, 42, 0.96);
      padding: 16px 18px;
      border-radius: 18px;
      box-shadow: 0 16px 40px rgba(0,0,0,0.55);
      border: 1px solid rgba(148, 163, 184, 0.35);
      backdrop-filter: blur(14px);
    }
    .card + .card {
      margin-top: 4px;
    }
    .card-header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 8px;
    }
    .card-title {
      font-size: 1.05rem;
      font-weight: 500;
      display: flex;
      align-items: center;
      gap: 6px;
    }
    .card-title span.emoji {
      font-size: 1.1rem;
    }
    .badge {
      font-size: 0.75rem;
      padding: 2px 8px;
      border-radius: 999px;
      border: 1px solid rgba(148, 163, 184, 0.7);
      color: #e5e7eb;
    }
    .badge-ok {
      border-color: #22c55e;
      color: #bbf7d0;
    }
    .badge-warn {
      border-color: #f97316;
      color: #fed7aa;
    }
    .badge-danger {
      border-color: #ef4444;
      color: #fecaca;
    }
    .badge-muted {
      border-color: #6b7280;
      color: #d1d5db;
    }
    .speed-row {
      display: flex;
      align-items: baseline;
      gap: 4px;
    }
    .speed-value {
      font-size: 2.7rem;
      font-weight: 600;
      letter-spacing: 0.03em;
    }
    .speed-unit {
      font-size: 0.9rem;
      color: #9ca3af;
      margin-left: 2px;
    }
    .row {
      display: flex;
      justify-content: space-between;
      align-items: baseline;
      margin-top: 6px;
      gap: 10px;
    }
    .label {
      font-size: 0.8rem;
      color: #9ca3af;
    }
    .value {
      font-size: 0.95rem;
      font-weight: 500;
      color: #e5e7eb;
      text-align: right;
      word-break: break-all;
    }
    .distance {
      font-size: 1.05rem;
      font-weight: 500;
      margin-top: 4px;
    }
    .btn {
      display:inline-block;
      margin-top:10px;
      padding:9px 14px;
      border-radius:999px;
      border:none;
      background:#3b82f6;
      color:#f9fafb;
      text-decoration:none;
      font-size:0.9rem;
      font-weight:500;
      text-align:center;
      cursor:pointer;
      box-shadow:0 4px 12px rgba(59,130,246,0.5);
      transition: transform 0.08s ease, box-shadow 0.08s ease, background 0.08s ease;
      width:100%;
    }
    .btn:hover {
      transform: translateY(-1px);
      box-shadow:0 6px 16px rgba(59,130,246,0.6);
      background:#2563eb;
    }
    .btn.disabled {
      background:#4b5563;
      box-shadow:none;
      cursor:default;
      opacity:0.7;
      transform:none;
    }
    .input {
      width: 100%;
      padding: 6px 9px;
      border-radius: 999px;
      border: 1px solid rgba(148,163,184,0.6);
      background: rgba(15,23,42,0.8);
      color: #e5e7eb;
      font-size: 0.9rem;
      outline: none;
    }
    .input:focus {
      border-color: #3b82f6;
      box-shadow:0 0 0 1px rgba(59,130,246,0.6);
    }
    .hint {
      font-size: 0.75rem;
      color: #9ca3af;
      margin-top: 2px;
      text-align: right;
    }
    .footer {
      margin-top:auto;
      text-align:center;
      font-size:0.75rem;
      color:#9ca3af;
      opacity:0.85;
      padding-top:6px;
    }
  </style>
</head>
<body>
  <div class="wrapper">
    <div class="header">
      <div class="title">Smart Bike Monitor</div>
      <div class="subtitle">Live speed, GPS & SMS via phone gateway</div>
      <div class="chip-row">
        <div class="chip">ESP32-C3 AP: BikeMonitor</div>
        <div class="chip">NEO-6M GPS</div>
        <div class="chip">SMS via Phone HTTP</div>
      </div>
    </div>

    <!-- SPEED CARD -->
    <div class="card">
      <div class="card-header">
        <div class="card-title">
          <span class="emoji">⚡</span>
          <span>Speed</span>
        </div>
        <div id="speedBadge" class="badge badge-muted">Idle</div>
      </div>
      <div class="speed-row">
        <span class="speed-value" id="speed">--.-</span>
        <span class="speed-unit">km/h</span>
      </div>
    </div>

    <!-- LOCATION CARD -->
    <div class="card">
      <div class="card-header">
        <div class="card-title">
          <span class="emoji">📍</span>
          <span>Location (GPS)</span>
        </div>
        <div id="gpsBadge" class="badge badge-warn">No fix</div>
      </div>

      <div class="row">
        <div class="label">Status</div>
        <div class="value" id="gpsStatus">Searching...</div>
      </div>

      <div class="row">
        <div class="label">Latitude</div>
        <div class="value" id="lat">--.--------</div>
      </div>

      <div class="row">
        <div class="label">Longitude</div>
        <div class="value" id="lon">--.--------</div>
      </div>

      <div class="row">
        <div class="label">Satellites</div>
        <div class="value" id="sats">--</div>
      </div>

      <div class="row">
        <div class="label">HDOP</div>
        <div class="value" id="hdop">--</div>
      </div>

      <div class="row" style="margin-top:10px;">
        <div class="label">Distance from home</div>
        <div class="distance" id="distance">--</div>
      </div>

      <a id="mapsLink" class="btn disabled" href="#" target="_blank">
        Open in Google Maps (GPS)
      </a>
    </div>

    <!-- PARENT & GATEWAY CONFIG CARD -->
    <div class="card">
      <div class="card-header">
        <div class="card-title">
          <span class="emoji">📲</span>
          <span>Parent & SMS Gateway</span>
        </div>
        <div id="configStatus" class="badge badge-muted">Not saved</div>
      </div>

      <div class="row">
        <div class="label">Parent phone</div>
        <div class="value" style="flex:1;">
          <input id="phoneInput" class="input" placeholder="+639XXXXXXXXX">
          <div class="hint">Used for SMS alerts (overspeed + status)</div>
        </div>
      </div>

      <div class="row" style="margin-top:6px;">
        <div class="label">Gateway IP</div>
        <div class="value" style="flex:1;">
          <input id="gwIpInput" class="input" placeholder="192.168.4.2">
          <div class="hint">Phone IP on Wi-Fi "BikeMonitor"</div>
        </div>
      </div>

      <div class="row" style="margin-top:6px;">
        <div class="label">Gateway port</div>
        <div class="value" style="flex:1;">
          <input id="gwPortInput" class="input" placeholder="8080">
          <div class="hint">Port of SMS app HTTP server</div>
        </div>
      </div>

      <button class="btn" id="configSaveBtn" onclick="saveConfig()">
        Save SMS settings
      </button>
    </div>

    <div class="footer">
       Connect to Wi-Fi "<b>BikeMonitor</b>" · Open <b>http://192.168.4.1</b> · Configure gateway & phone
    </div>
  </div>

  <script>
    function describeGpsStatus(d) {
      const sats = d.sats || 0;
      const hdop = (typeof d.hdop === 'number')
        ? d.hdop
        : parseFloat(d.hdop || '99');

      if (!d.gpsFix) {
        if (sats === 0) {
          return "GPS is warming up… go somewhere with clear sky.";
        } else if (sats < 4) {
          return `Seeing ${sats} satellite${sats === 1 ? "" : "s"}, waiting for a stable fix…`;
        } else {
          return `Almost there: ${sats} satellites visible, stabilizing location…`;
        }
      }

      let quality = "";
      if (hdop <= 1.5) quality = "Very accurate location";
      else if (hdop <= 3.0) quality = "Good location";
      else if (hdop <= 6.0) quality = "OK but not perfect";
      else quality = "Weak GPS signal";

      return `${quality} (${sats} satellites, HDOP ${isNaN(hdop) ? "?" : hdop.toFixed(1)}).`;
    }

    async function saveConfig() {
      const phoneInput = document.getElementById('phoneInput');
      const gwIpInput = document.getElementById('gwIpInput');
      const gwPortInput = document.getElementById('gwPortInput');
      const status = document.getElementById('configStatus');
      const btn = document.getElementById('configSaveBtn');

      const phone = (phoneInput.value || "").trim();
      const ip = (gwIpInput.value || "").trim();
      const port = (gwPortInput.value || "").trim();

      if (!phone || !ip || !port) {
        status.textContent = "Missing data";
        status.className = "badge badge-danger";
        return;
      }

      btn.classList.add('disabled');
      btn.disabled = true;

      try {
        const url = `/setConfig?phone=${encodeURIComponent(phone)}&ip=${encodeURIComponent(ip)}&port=${encodeURIComponent(port)}`;
        const res = await fetch(url);
        if (res.ok) {
          status.textContent = "Saved";
          status.className = "badge badge-ok";
        } else {
          status.textContent = "Error saving";
          status.className = "badge badge-danger";
        }
      } catch (e) {
        console.log(e);
        status.textContent = "Network error";
        status.className = "badge badge-danger";
      }

      btn.classList.remove('disabled');
      btn.disabled = false;
    }

    async function updateData() {
      try {
        const res = await fetch('/data');
        const d = await res.json();

        // ----- SPEED -----
        const speedElem = document.getElementById('speed');
        const speedBadge = document.getElementById('speedBadge');
        const speed = d.speed;

        speedElem.textContent = speed.toFixed(1);

        speedBadge.className = "badge badge-muted";
        if (speed < 0.5) {
          speedBadge.textContent = 'Idle';
        } else if (d.overspeed) {
          speedBadge.textContent = 'Overspeed';
          speedBadge.className = "badge badge-danger";
        } else {
          speedBadge.textContent = 'Moving';
          speedBadge.className = "badge badge-ok";
        }

        // ----- GPS STATUS + COORDS -----
        const gpsBadge = document.getElementById('gpsBadge');
        const gpsStatus = document.getElementById('gpsStatus');
        const latElem = document.getElementById('lat');
        const lonElem = document.getElementById('lon');

        gpsBadge.className = "badge";
        const sats = d.sats || 0;
        const hdop = (typeof d.hdop === 'number')
          ? d.hdop
          : parseFloat(d.hdop || '99');

        gpsStatus.textContent = describeGpsStatus(d);

        if (!d.gpsFix) {
          gpsBadge.textContent = 'No fix';
          gpsBadge.classList.add('badge-warn');
          latElem.textContent = '--.--------';
          lonElem.textContent = '--.--------';
        } else {
          gpsBadge.textContent = 'Fix OK';
          gpsBadge.classList.add('badge-ok');
          latElem.textContent = d.lat.toFixed(6);
          lonElem.textContent = d.lon.toFixed(6);
        }

        document.getElementById('sats').textContent = sats;
        document.getElementById('hdop').textContent =
          isNaN(hdop) ? '--' : hdop.toFixed(1);

        // ----- DISTANCE FROM HOME -----
        const distanceElem = document.getElementById('distance');
        let distText = '--';
        if (d.gpsFix) {
          if (d.distanceM < 1000) {
            distText = d.distanceM.toFixed(0) + ' m';
          } else {
            distText = (d.distanceM / 1000.0).toFixed(2) + ' km';
          }
        }
        distanceElem.textContent = distText;

        // ----- GOOGLE MAPS LINK (real GPS) -----
        const link = document.getElementById('mapsLink');
        if (d.gpsFix && d.mapsUrl && d.mapsUrl.length > 0) {
          link.href = d.mapsUrl;
          link.classList.remove('disabled');
          link.textContent = 'Open in Google Maps (GPS)';
        } else {
          link.href = '#';
          link.classList.add('disabled');
          link.textContent = 'Waiting for GPS location…';
        }

        // ----- CONFIG FIELDS (fill once) -----
        const phoneInput = document.getElementById('phoneInput');
        const gwIpInput = document.getElementById('gwIpInput');
        const gwPortInput = document.getElementById('gwPortInput');
        const configStatus = document.getElementById('configStatus');

        if (!phoneInput.dataset.initialized) {
          if (d.parentPhone) phoneInput.value = d.parentPhone;
          if (d.gatewayHost) gwIpInput.value = d.gatewayHost;
          if (d.gatewayPort) gwPortInput.value = d.gatewayPort;
          phoneInput.dataset.initialized = "1";
          configStatus.textContent = "Loaded";
          configStatus.className = "badge badge-ok";
        }
      } catch (e) {
        console.log('Error fetching data', e);
      }
    }

    setInterval(updateData, 1000);
    updateData();
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleData() {
  bool fix = gps.location.isValid();
  double lat = fix ? gps.location.lat() : 0.0;
  double lon = fix ? gps.location.lng() : 0.0;
  uint32_t sats = gps.satellites.value();
  double hdopVal = gps.hdop.hdop();  // 0 if not valid

  double distM = 0.0;
  if (fix) {
    distM = distanceMeters(HOME_LAT, HOME_LON, lat, lon);
  }

  bool overspeed = currentSpeedKmh > SPEED_LIMIT_KMH;

  String mapsUrl = "";
  if (fix) {
    mapsUrl = "https://www.google.com/maps?q=" +
              String(lat, 6) + "," + String(lon, 6);
  }

  String json = "{";
  json += "\"speed\":" + String(currentSpeedKmh, 2);
  json += ",\"overspeed\":" + String(overspeed ? "true" : "false");
  json += ",\"gpsFix\":" + String(fix ? "true" : "false");
  json += ",\"lat\":" + String(lat, 6);
  json += ",\"lon\":" + String(lon, 6);
  json += ",\"sats\":" + String(sats);
  json += ",\"hdop\":" + String(hdopVal, 1);
  json += ",\"distanceM\":" + String(distM, 1);
  json += ",\"mapsUrl\":\"" + mapsUrl + "\"";
  json += ",\"parentPhone\":\"" + parentPhone + "\"";
  json += ",\"gatewayHost\":\"" + gatewayHost + "\"";
  json += ",\"gatewayPort\":" + String(gatewayPort);
  json += ",\"fixedMapUrl\":\"" + String(FIXED_MAP_URL) + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleSetConfig() {
  if (!server.hasArg("phone") || !server.hasArg("ip") || !server.hasArg("port")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing params\"}");
    return;
  }

  parentPhone = server.arg("phone");
  parentPhone.trim();

  gatewayHost = server.arg("ip");
  gatewayHost.trim();

  gatewayPort = (uint16_t) server.arg("port").toInt();

  server.send(200, "application/json", "{\"ok\":true}");
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ---------------------- SETUP -----------------------------

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nSmart Bike - AP + GPS + Web UI + SMS via Phone Gateway (fixed map link)");

  // Hall sensor input
  pinMode(HALL_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  // GPS serial
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started at 9600 baud");

  // Wi-Fi Access Point
  WiFi.mode(WIFI_AP);

  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGW(192, 168, 4, 1);
  IPAddress apMask(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, apGW, apMask);

  bool apStarted = WiFi.softAP(AP_SSID, AP_PASS);
  if (apStarted) {
    Serial.print("AP started. SSID: ");
    Serial.print(AP_SSID);
    Serial.print("  Password: ");
    Serial.println(AP_PASS);
    Serial.print("ESP32 AP IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Failed to start AP!");
  }

  // Web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/setConfig", handleSetConfig);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

// ---------------------- LOOP ------------------------------

void loop() {
  // Update sensors
  readGPS();
  updateSpeed();

  // SMS logic
  maybeSendStartupSms();
  handleSmsLogic();

  // Handle web requests
  server.handleClient();

  // Optional serial debug
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;

    Serial.println("---------------");
    Serial.print("Speed: ");
    Serial.print(currentSpeedKmh, 2);
    Serial.println(" km/h");

    if (gps.location.isValid()) {
      double lat = gps.location.lat();
      double lon = gps.location.lng();
      Serial.print("Lat: ");
      Serial.print(lat, 6);
      Serial.print("  Lon: ");
      Serial.println(lon, 6);

      Serial.print("Satellites: ");
      Serial.println(gps.satellites.value());
      Serial.print("HDOP: ");
      Serial.println(gps.hdop.hdop());

      double distM = distanceMeters(HOME_LAT, HOME_LON, lat, lon);
      Serial.print("Distance from home: ");
      if (distM < 1000) {
        Serial.print(distM, 0);
        Serial.println(" m");
      } else {
        Serial.print(distM / 1000.0, 2);
        Serial.println(" km");
      }

      String mapsUrl = "https://www.google.com/maps?q=" +
                       String(lat, 6) + "," + String(lon, 6);
      Serial.print("Maps URL (GPS): ");
      Serial.println(mapsUrl);

    } else {
      Serial.println("GPS: No valid fix yet");
    }

    Serial.print("Parent phone: ");
    Serial.println(parentPhone);
    Serial.print("Gateway: ");
    Serial.print(gatewayHost);
    Serial.print(":");
    Serial.println(gatewayPort);
  }
}
