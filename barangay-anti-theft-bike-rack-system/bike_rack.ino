/*
  MAIN ESP32-WROOM-32
  Barangay Bike Rack System (2 racks)

  - WiFi AP + web UI
  - LCD + keypad self-service (keypad on ESP32-C3 via UART)
  - RFID control (RC522 on right-side VSPI pins)
  - Limit switches for bike presence + tamper detection
  - SMS alerts via phone gateway
  - 2 linear actuators driven by 4 SPDT relays (2 per actuator)
  - Buzzer via GPIO27 (through relay or transistor)

  LINEAR ACTUATORS (per rack):
    - Each actuator has 2 wires.
    - Each actuator is driven by 2 SPDT relays (A and B) as an H-bridge.

    Wiring per actuator (12 V side):
      Relay A COM -> actuator wire 1
      Relay B COM -> actuator wire 2
      Relay A NO  -> +12 V
      Relay A NC  -> GND
      Relay B NO  -> +12 V
      Relay B NC  -> GND

    Logic in this code:
      - LOCK   -> actuator EXTEND  (A ON, B OFF) for 8 seconds, then both OFF
      - UNLOCK -> actuator RETRACT (A OFF, B ON) for 8 seconds, then both OFF
      - STOP   -> both OFF

  KEYPAD:
    - Scanned by ESP32-C3.
    - C3 sends key chars via UART to this ESP32.
    - C3 TX -> WROOM GPIO16 (RX2).

  LIMIT SWITCHES (bike presence):
    SW1_PIN (Rack 1) = GPIO25
    SW2_PIN (Rack 2) = GPIO26
    Wiring: switch between pin and GND, INPUT_PULLUP, pressed=LOW means bike present.

  BUZZER:
    BUZZER_PIN = GPIO27
    Assumes active-HIGH: digitalWrite(HIGH) turns buzzer ON via relay/transistor.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClient.h>

#include <SPI.h>
#include <MFRC522.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =====================================================
//                  WiFi AP + Web Auth
// =====================================================
const char* AP_SSID  = "EARIST_BRGY_BIKE_RACK";
const char* AP_PASS  = "Barangay#2026";
const char* WEB_USER = "barangay";
const char* WEB_PASS = "earist2026";

WebServer server(80);

bool checkAuth() {
  if (!server.authenticate(WEB_USER, WEB_PASS)) {
    server.requestAuthentication();
    return false;
  }
  return true;
}

// =====================================================
//           PHONE SMS GATEWAY (Simple SMS Gateway app)
// =====================================================
const char* SMSGW_HOST = "192.168.4.2";
const uint16_t SMSGW_PORT = 8080;
const char* SMSGW_PATH = "/send-sms";
const char* SMSGW_TOKEN = "";
const char* SMSGW_TOKEN_HEADER = "X-Api-Key";

// 30s cooldown per rack to avoid spam
unsigned long lastSmsMillis[2] = {0, 0};
const unsigned long SMS_COOLDOWN_MS = 30000;

// =====================================================
//                        LCD I2C
// =====================================================
LiquidCrystal_I2C lcd(0x27, 16, 2); // try 0x3F if not working

void lcd2(const String& a, const String& b) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(a);
  lcd.setCursor(0,1); lcd.print(b);
}

// =====================================================
//                        RFID (RC522) on VSPI
// =====================================================
// VSPI pins: SCK=18, MISO=19, MOSI=23
#define RFID_SS   5
#define RFID_RST  15
MFRC522 rfid(RFID_SS, RFID_RST);

const byte RACK1_UID[] = {0xDA, 0xD9, 0xFB, 0xA9 };
const byte RACK2_UID[] = {0xCB, 0x4F, 0xB0, 0x05};
const byte UID_LEN = 4;

// Keep RC522 quiet during ESP32 boot
void rfidSafeBootInit() {
  pinMode(RFID_SS, OUTPUT);
  pinMode(RFID_RST, OUTPUT);

  digitalWrite(RFID_SS, HIGH);   // SPI idle
  digitalWrite(RFID_RST, LOW);   // hold RC522 in reset during ESP32 boot

  delay(1200);

  SPI.begin(18, 19, 23, RFID_SS);

  digitalWrite(RFID_RST, HIGH);
  delay(50);

  rfid.PCD_Init();
}

// =====================================================
//            ACTUATOR RELAYS (4 SPDT, 2 per actuator)
// =====================================================
// Chosen to avoid conflicts with RC522, switches, buzzer, UART

// Rack 1 actuator: GPIO13, GPIO14
// Rack 2 actuator: GPIO32, GPIO33

const int R1_RELAY_A_PIN = 13;   // ONE SIDE of Rack 1 actuator
const int R1_RELAY_B_PIN = 14;   // OTHER SIDE of Rack 1 actuator

const int R2_RELAY_A_PIN = 32;   // ONE SIDE of Rack 2 actuator
const int R2_RELAY_B_PIN = 33;   // OTHER SIDE of Rack 2 actuator

// Relay logic type (typical modules: ACTIVE-LOW)
const bool RELAY_ACTIVE_LOW = true;

// Movement time for full extend/retract
const unsigned long ACTUATOR_MOVE_TIME_MS = 8000;   // 8 seconds

int relayOnLevel()  { return RELAY_ACTIVE_LOW ? LOW : HIGH; }
int relayOffLevel() { return RELAY_ACTIVE_LOW ? HIGH : LOW; }

enum MotionState {
  MOTION_IDLE = 0,
  MOTION_EXTEND,
  MOTION_RETRACT
};

struct RackMotion {
  MotionState state;
  unsigned long startMs;
};

RackMotion rackMotion[2];

// Low-level relay patterns --------------------------------
void rackSetStop(int rack) {
  if (rack == 0) {
    digitalWrite(R1_RELAY_A_PIN, relayOffLevel());
    digitalWrite(R1_RELAY_B_PIN, relayOffLevel());
  } else {
    digitalWrite(R2_RELAY_A_PIN, relayOffLevel());
    digitalWrite(R2_RELAY_B_PIN, relayOffLevel());
  }
}

void rackSetExtend(int rack) {
  // LOCK = actuator EXTEND
  if (rack == 0) {
    digitalWrite(R1_RELAY_A_PIN, relayOnLevel());
    digitalWrite(R1_RELAY_B_PIN, relayOffLevel());
  } else {
    digitalWrite(R2_RELAY_A_PIN, relayOnLevel());
    digitalWrite(R2_RELAY_B_PIN, relayOffLevel());
  }
}

void rackSetRetract(int rack) {
  // UNLOCK = actuator RETRACT
  if (rack == 0) {
    digitalWrite(R1_RELAY_A_PIN, relayOffLevel());
    digitalWrite(R1_RELAY_B_PIN, relayOnLevel());
  } else {
    digitalWrite(R2_RELAY_A_PIN, relayOffLevel());
    digitalWrite(R2_RELAY_B_PIN, relayOnLevel());
  }
}

void allStop() {
  rackSetStop(0);
  rackSetStop(1);
  rackMotion[0].state = MOTION_IDLE;
  rackMotion[1].state = MOTION_IDLE;
}

// Start motion for one rack
// lockedTarget: true = LOCK (extend), false = UNLOCK (retract)
void startMotionForRack(int rack, bool lockedTarget) {
  RackMotion &rm = rackMotion[rack];

  // Small safety stop before reversing direction
  rackSetStop(rack);
  delay(50);

  rm.startMs = millis();

  if (lockedTarget) {
    rm.state = MOTION_EXTEND;
    rackSetExtend(rack);
  } else {
    rm.state = MOTION_RETRACT;
    rackSetRetract(rack);
  }
}

// Called periodically from loop()
void updateMotion() {
  unsigned long now = millis();

  for (int rack = 0; rack < 2; rack++) {
    RackMotion &rm = rackMotion[rack];

    if (rm.state == MOTION_IDLE) continue;

    if (now - rm.startMs >= ACTUATOR_MOVE_TIME_MS) {
      rackSetStop(rack);
      rm.state = MOTION_IDLE;
    }
  }
}

// =====================================================
//                        BUZZER
// =====================================================
const int BUZZER_PIN = 27;      // via relay/transistor; HIGH = ON (adjust if needed)
unsigned long buzzerOffAtMs = 0;

void buzzerOff() {
  digitalWrite(BUZZER_PIN, LOW);
  buzzerOffAtMs = 0;
}

void buzzerOnFor(unsigned long ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerOffAtMs = millis() + ms;
}

void updateBuzzer() {
  if (buzzerOffAtMs && millis() >= buzzerOffAtMs) {
    buzzerOff();
  }
}

// =====================================================
//          LIMIT SWITCHES (2 racks) – bike presence
// =====================================================
// Switch to GND, INPUT_PULLUP at MCU.
// Pressed = LOW => bike present
const int SW1_PIN = 25;
const int SW2_PIN = 26;

#define BIKE_PRESENT_LEVEL LOW

bool bikePresent[2] = {false, false};
bool lastBikePresent[2] = {false, false};

unsigned long lastSwChangeMs[2] = {0, 0};
const unsigned long SW_DEBOUNCE_MS = 60;

// =====================================================
//                       RACK STATE
// =====================================================
String lastEvent = "Boot";

struct Rack {
  String ownerName;
  String ownerNumber;
  bool locked = false;

  const byte* tagUid = nullptr;
  byte tagLen = 0;

  bool tamperLatched = false;
};

Rack racks[2];

// =====================================================
//                 KEYPAD LINK (ESP32-C3 -> UART2)
// =====================================================
// C3 sends one char per keypress over UART.
// C3 TX -> WROOM GPIO16 (RX2)

HardwareSerial &KeypadSerial = Serial2;
const int KEYPAD_UART_RX_PIN = 16;
const unsigned long KEYPAD_BAUD = 115200;

char readRemoteKey() {
  if (KeypadSerial.available() > 0) {
    return (char)KeypadSerial.read();
  }
  return 0;
}

// =====================================================
//                       EVENT LOG (RAM)
// =====================================================
struct LogEntry {
  uint32_t ms;
  String msg;
};

const int LOG_MAX = 80;
LogEntry logs[LOG_MAX];
int logHead = 0;
int logCount = 0;

String formatUptime(uint32_t ms) {
  uint32_t s = ms / 1000;
  uint32_t days = s / 86400; s %= 86400;
  uint32_t hrs  = s / 3600;  s %= 3600;
  uint32_t mins = s / 60;    s %= 60;
  uint32_t secs = s;

  char buf[24];
  snprintf(buf, sizeof(buf), "%lu %02lu:%02lu:%02lu",
           (unsigned long)days, (unsigned long)hrs,
           (unsigned long)mins, (unsigned long)secs);
  return String(buf);
}

String maskNumber(const String& n) {
  if (n.length() != 11) return n;
  return n.substring(0, 2) + "*******" + n.substring(9);
}

void addLog(const String& message) {
  logs[logHead].ms = millis();
  logs[logHead].msg = message;
  logHead = (logHead + 1) % LOG_MAX;
  if (logCount < LOG_MAX) logCount++;
}

String logsJson() {
  String out = "[";
  for (int i = 0; i < logCount; i++) {
    int idx = (logHead - 1 - i);
    while (idx < 0) idx += LOG_MAX;
    idx %= LOG_MAX;

    String ts = formatUptime(logs[idx].ms);
    String msg = logs[idx].msg;
    msg.replace("\\", "\\\\");
    msg.replace("\"", "\\\"");
    msg.replace("\n", "\\n");

    if (i) out += ",";
    out += "{\"t\":\"" + ts + "\",\"msg\":\"" + msg + "\"}";
  }
  out += "]";
  return out;
}

// =====================================================
//                Self-service UI state machine
// =====================================================
enum UiState {
  UI_IDLE,
  UI_ENTER_NUMBER,
  UI_WAIT_RFID_LOCK
};

UiState uiState = UI_IDLE;
int uiSelectedRack = -1;
String uiNum = "";

// =====================================================
//                   Helper: UID functions
// =====================================================
String uidToString(const byte* uid, byte len) {
  String s;
  for (byte i = 0; i < len; i++) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
    if (i != len - 1) s += " ";
  }
  s.toUpperCase();
  return s;
}

bool uidEquals(const byte* a, byte aLen, const byte* b, byte bLen) {
  if (aLen != bLen) return false;
  for (byte i = 0; i < aLen; i++) if (a[i] != b[i]) return false;
  return true;
}

bool digitsOnly(const String& s) {
  for (size_t i = 0; i < s.length(); i++) if (s[i] < '0' || s[i] > '9') return false;
  return true;
}

// =====================================================
//        PHONE SMS GATEWAY: HTTP send (JSON POST)
// =====================================================
String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '\\' || c == '"') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') { /* skip */ }
    else out += c;
  }
  return out;
}

bool sendViaPhoneGateway(const String& number, const String& message) {
  if (number.length() != 11) return false;
  if (WiFi.getMode() != WIFI_AP) return false;

  WiFiClient client;
  HTTPClient http;

  String url = String("http://") + SMSGW_HOST + ":" + String(SMSGW_PORT) + SMSGW_PATH;

  if (!http.begin(client, url)) return false;

  http.setTimeout(8000);
  http.addHeader("Content-Type", "application/json");

  if (String(SMSGW_TOKEN).length() > 0) {
    http.addHeader(SMSGW_TOKEN_HEADER, SMSGW_TOKEN);
  }

  String body = "{";
  body += "\"to\":\"" + jsonEscape(number) + "\",";
  body += "\"message\":\"" + jsonEscape(message) + "\"";
  body += "}";

  int code = http.POST(body);
  String resp = http.getString();
  http.end();

  return (code >= 200 && code < 300);
}

bool sendAlertSMS(int rackIdx, const String& number, const String& message) {
  unsigned long now = millis();
  if (now - lastSmsMillis[rackIdx] < SMS_COOLDOWN_MS) return false;

  bool ok = sendViaPhoneGateway(number, message);
  if (ok) lastSmsMillis[rackIdx] = now;
  return ok;
}

// =====================================================
//                   Lock control (local relays)
// =====================================================
void setLocked(int rackIndex, bool lockIt) {
  racks[rackIndex].locked = lockIt;
  if (!lockIt) racks[rackIndex].tamperLatched = false;

  // Trigger actuator movement for this rack
  startMotionForRack(rackIndex, lockIt);
}

void toggleLock(int rackIndex) {
  setLocked(rackIndex, !racks[rackIndex].locked);
}

// =====================================================
//                  Limit switch reading + tamper
// =====================================================
bool readBikePresent(int pin) {
  // INPUT_PULLUP + switch to GND => pressed=LOW
  return (digitalRead(pin) == BIKE_PRESENT_LEVEL);
}

void updateBikePresence() {
  bool sw1 = readBikePresent(SW1_PIN);
  bool sw2 = readBikePresent(SW2_PIN);

  unsigned long now = millis();
  bool rawPresent[2] = { sw1, sw2 };

  for (int i = 0; i < 2; i++) {
    if (rawPresent[i] != bikePresent[i]) {
      if (now - lastSwChangeMs[i] >= SW_DEBOUNCE_MS) {
        lastSwChangeMs[i] = now;
        bikePresent[i] = rawPresent[i];
      }
    }
  }
}

void checkTamper() {
  for (int i = 0; i < 2; i++) {
    bool bikeJustRemoved = lastBikePresent[i] && !bikePresent[i];

    if (bikeJustRemoved && racks[i].locked && !racks[i].tamperLatched) {
      racks[i].tamperLatched = true;

      String owner = racks[i].ownerName.length() ? racks[i].ownerName : "Owner";
      String msg = "ALERT! Someone is trying to steal the bike on Rack " + String(i + 1) +
                   ". Bike was lifted while LOCKED. (" + owner + ")";

      bool ok = false;
      if (racks[i].ownerNumber.length() == 11) {
        ok = sendAlertSMS(i, racks[i].ownerNumber, msg);
        lastEvent = ok ? ("SMS sent: Tamper Rack " + String(i + 1))
                       : ("SMS FAIL/COOLDOWN: Tamper Rack " + String(i + 1));
      } else {
        lastEvent = "Tamper but owner # missing (Rack " + String(i + 1) + ")";
      }

      addLog(String("TAMPER Rack ") + (i + 1) + " Number " +
             maskNumber(racks[i].ownerNumber) + (ok ? " SMS_OK" : " SMS_FAIL"));

      lcd2("ALERT TAMPER!", "Rack " + String(i + 1));

      // Beep buzzer for 3 seconds
      buzzerOnFor(3000);
    }

    lastBikePresent[i] = bikePresent[i];
  }
}

// =====================================================
//                Keypad + LCD Self-service logic
// =====================================================
bool anySelfServeCandidate() {
  for (int i = 0; i < 2; i++) {
    if (bikePresent[i] && !racks[i].locked) return true;
  }
  return false;
}

// Pick the most recently "touched" rack that has a bike and is not locked
int findNewestAvailableRack() {
  int best = -1;
  unsigned long bestTime = 0;

  for (int i = 0; i < 2; i++) {
    if (bikePresent[i] && !racks[i].locked) {
      if (best == -1 || lastSwChangeMs[i] > bestTime) {
        best = i;
        bestTime = lastSwChangeMs[i];
      }
    }
  }
  return best; // -1 = no suitable rack
}

bool startSelfServeForCurrentBike() {
  int rack = findNewestAvailableRack();
  if (rack == -1) {
    lcd2("No Slot", "Please Wait");
    delay(2000);
    uiShowIdle();
    return false;
  }

  uiSelectedRack = rack;
  uiNum = "";
  uiState = UI_ENTER_NUMBER;
  uiShowEnterNum();
  return true;
}

void uiShowIdle() { lcd2("Place bike then", "Press # to start"); }

void uiShowEnterNum() {
  String line1 = "Rack " + String(uiSelectedRack + 1) + " Type CP #:";
  String line2 = uiNum;
  if (line2.length() > 16) line2 = line2.substring(line2.length() - 16);
  lcd2(line1, line2);
}

void uiShowWaitRFID() { lcd2("Tap RFID to", "LOCK Rack " + String(uiSelectedRack + 1)); }

void resetUi() {
  uiState = UI_IDLE;
  uiSelectedRack = -1;
  uiNum = "";
  uiShowIdle();
}

void handleSelfServeKeypad() {
  char k = readRemoteKey();
  if (!k) return;

  if (uiState == UI_IDLE) {
    if (k == '#') {
      // Auto-detect rack instead of asking 1 or 2
      startSelfServeForCurrentBike();
    }
    return;
  }

  if (uiState == UI_ENTER_NUMBER) {
    if (k >= '0' && k <= '9') {
      if (uiNum.length() < 11) uiNum += k;
      uiShowEnterNum();
      return;
    }

    if (k == '*') {
      if (uiNum.length() > 0) {
        // backspace one digit
        uiNum.remove(uiNum.length() - 1);
        uiShowEnterNum();
      } else {
        // no digits -> cancel session
        resetUi();
      }
      return;
    }

    if (k == '#') {
      if (uiNum.length() != 11 || !digitsOnly(uiNum)) {
        lcd2("Invalid number", "Need 11 digits");
        delay(900);
        uiShowEnterNum();
        return;
      }

      racks[uiSelectedRack].ownerNumber = uiNum;
      if (racks[uiSelectedRack].ownerName.length() == 0) racks[uiSelectedRack].ownerName = "Self-Serve";

      lastEvent = "Self-serve set # (Rack " + String(uiSelectedRack + 1) + ")";
      addLog("SELF-SERVE Rack " + String(uiSelectedRack + 1) + " Number " + maskNumber(uiNum));

      uiState = UI_WAIT_RFID_LOCK;
      uiShowWaitRFID();
      return;
    }
    return;
  }

  if (uiState == UI_WAIT_RFID_LOCK) {
    if (k == '*') {
      // Cancel lock process completely
      resetUi();
    }
    return;
  }
}

// =====================================================
//                   RFID logic + Unknown RFID SMS
// =====================================================
void alertUnknownRFID(const String& uidStr) {
  String msg = "ALERT: Unknown RFID near bike rack. UID=" + uidStr;

  bool anySent = false;
  for (int i = 0; i < 2; i++) {
    if (racks[i].locked && racks[i].ownerNumber.length() == 11) {
      bool ok = sendAlertSMS(i, racks[i].ownerNumber, msg);
      anySent = anySent || ok;
    }
  }

  addLog("UNKNOWN RFID UalertID=" + uidStr);

  lastEvent = anySent ? "SMS sent: Unknown RFID (locked racks)"
                      : "Unknown RFID (no locked owners / cooldown / gateway fail)";
}

void handleRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  byte* uid = rfid.uid.uidByte;
  byte  len = rfid.uid.size;
  String uidStr = uidToString(uid, len);

  bool matched = false;

  if (uiState == UI_WAIT_RFID_LOCK && uiSelectedRack != -1) {
    int i = uiSelectedRack;
    if (uidEquals(uid, len, racks[i].tagUid, racks[i].tagLen)) {
      setLocked(i, true);
      matched = true;
      lastEvent = "RFID LOCK Rack " + String(i + 1);

      addLog("LOCK Rack " + String(i + 1) + " (RFID) Number " + maskNumber(racks[i].ownerNumber));

      lcd2("Rack " + String(i + 1) + " LOCKED", "Remove key/card");
      delay(2000);
      resetUi();
    } else {
      lcd2("Wrong RFID", "Use Rack tag");
      delay(2000);
      uiShowWaitRFID();
    }

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
    delay(200);
    return;
  }

  for (int i = 0; i < 2; i++) {
    if (uidEquals(uid, len, racks[i].tagUid, racks[i].tagLen)) {
      toggleLock(i);
      matched = true;
      lastEvent = "RFID toggle Rack " + String(i + 1) + (racks[i].locked ? " (LOCK)" : " (UNLOCK)");

      addLog(String("RFID ") + (racks[i].locked ? "LOCK " : "UNLOCK ") +
             "Rack " + String(i + 1) + " Number " + maskNumber(racks[i].ownerNumber));
      break;
    }
  }

  if (!matched) alertUnknownRFID(uidStr);

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  delay(200);
}

// =====================================================
//                      Web UI HTML
// =====================================================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Barangay Bike Rack (Dual)</title>
<style>
  :root{
    --bg1:#0b1220; --bg2:#111b33;
    --card: rgba(255,255,255,.05);
    --card2: rgba(255,255,255,.02);
    --line: rgba(255,255,255,.10);
    --muted:#94a3b8;
    --text:#e5e7eb;
    --good:#22c55e;
    --warn:#f59e0b;
    --bad:#ef4444;
  }
  *{ box-sizing:border-box; }
  body{
    margin:0;
    font-family: system-ui,-apple-system,Segoe UI,Roboto,Arial;
    color:var(--text);
    background:
      radial-gradient(1200px 800px at 15% 10%, #1f3b8a55, transparent 60%),
      radial-gradient(1000px 700px at 80% 0%,  #7c3aed44, transparent 55%),
      linear-gradient(180deg,var(--bg1),var(--bg2));
  }
  .wrap{ max-width:1100px; margin:0 auto; padding:18px; }
  .top{ display:flex; justify-content:space-between; align-items:flex-start; gap:14px; margin-bottom:14px; }
  h1{ margin:0; font-size:18px; letter-spacing:.2px; }
  .sub{ margin-top:6px; color:var(--muted); font-size:13px; line-height:1.35; }
  .statusPill{ display:flex; align-items:center; gap:10px; padding:10px 12px; border-radius:999px; border:1px solid var(--line); background: rgba(255,255,255,.04); color:var(--muted); font-size:13px; white-space:nowrap; }
  .dot{ width:10px; height:10px; border-radius:999px; background:var(--warn); }
  .grid{ display:grid; grid-template-columns:1fr; gap:14px; }
  @media(min-width:900px){ .grid{ grid-template-columns:1fr 1fr; } }
  .card{ border:1px solid var(--line); background: linear-gradient(180deg, var(--card), var(--card2)); border-radius:18px; padding:16px; box-shadow: 0 12px 30px rgba(0,0,0,.25); }
  .cardHead{ display:flex; align-items:center; justify-content:space-between; gap:12px; margin-bottom:10px; }
  .cardHead h2{ margin:0; font-size:14px; color:#cbd5e1; font-weight:700; letter-spacing:.2px; }
  .badges{ display:flex; gap:8px; flex-wrap:wrap; justify-content:flex-end; }
  .badge{ padding:6px 10px; border-radius:999px; border:1px solid var(--line); background: rgba(0,0,0,.18); font-size:12px; color:var(--muted); }
  .badge.good{ border-color: rgba(34,197,94,.35); color:#bbf7d0; }
  .badge.bad{ border-color: rgba(239,68,68,.40); color:#fecaca; }
  .badge.warn{ border-color: rgba(245,158,11,.45); color:#fde68a; }
  .kv{ display:grid; grid-template-columns: 1fr; gap:10px; margin-top:10px; }
  .row{ display:flex; justify-content:space-between; align-items:center; padding:10px 12px; border-radius:14px; background: rgba(0,0,0,.18); border:1px solid var(--line); }
  .k{ color:var(--muted); font-size:13px; }
  .v{ font-weight:800; font-size:13px; text-align:right; }
  .mono{ font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; }
  .form{ margin-top:12px; display:grid; grid-template-columns:1fr; gap:10px; }
  @media(min-width:900px){ .form{ grid-template-columns: 1fr 1fr; } }
  input{ width:100%; margin:0; padding:12px 12px; border-radius:14px; border:1px solid rgba(255,255,255,.18); background:#ffffff; color:#111827; outline:none; font-size:14px; }
  input::placeholder{ color:#6b7280; opacity:1; }
  .btnbar{ display:flex; flex-wrap:wrap; gap:10px; margin-top:12px; }
  button{ border:1px solid var(--line); background: linear-gradient(180deg, rgba(255,255,255,.08), rgba(255,255,255,.04)); color:var(--text); padding:12px 12px; border-radius:14px; cursor:pointer; font-weight:800; font-size:13px; }
  button:hover{ filter:brightness(1.06); }
  button.danger{ border-color: rgba(239,68,68,.45); background: linear-gradient(180deg, rgba(239,68,68,.25), rgba(239,68,68,.12)); }
  button.soft{ border-color: rgba(34,197,94,.35); background: linear-gradient(180deg, rgba(34,197,94,.20), rgba(34,197,94,.10)); }
  .logBox{ margin-top:10px; border:1px solid var(--line); background: rgba(0,0,0,.18); border-radius:14px; overflow:hidden; }
  .logHead{ display:flex; justify-content:space-between; align-items:center; gap:10px; padding:10px 12px; border-bottom:1px solid rgba(255,255,255,.08); background: rgba(255,255,255,.03); }
  .logTitle{ font-size:13px; font-weight:800; color:#cbd5e1; margin:0; }
  .logMeta{ font-size:12px; color:var(--muted); }
  .logList{ max-height:280px; overflow:auto; }
  .logRow{ display:grid; grid-template-columns: 118px 1fr; gap:10px; padding:10px 12px; border-bottom:1px solid rgba(255,255,255,.06); align-items:start; }
  .logRow:last-child{ border-bottom:none; }
  .ts{ font-size:12px; color:var(--muted); white-space:nowrap; font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; }
  .msg{ font-size:13px; line-height:1.35; color:var(--text); word-break:break-word; }
  details{ margin-top:12px; border:1px solid var(--line); border-radius:14px; background: rgba(0,0,0,.12); padding:10px 12px; }
  summary{ cursor:pointer; color:#cbd5e1; font-weight:800; font-size:13px; list-style:none; }
  summary::-webkit-details-marker{ display:none; }
  pre{ margin:10px 0 0 0; white-space:pre-wrap; word-break:break-word; color:#94a3b8; font-size:12px; }
</style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div>
      <h1>EARIST Public Bike Rack (Dual)</h1>
    </div>
    <div class="statusPill">
      <span class="dot" id="dot"></span>
      <span id="headline">Loading…</span>
    </div>
  </div>

  <div class="grid">
    <div class="card">
      <div class="cardHead">
        <h2>Rack 1</h2>
        <div class="badges">
          <span class="badge" id="r1_lock_badge">—</span>
          <span class="badge" id="r1_bike_badge">—</span>
        </div>
      </div>
      <div class="kv">
        <div class="row"><div class="k">Owner</div><div class="v mono" id="r1_owner">—</div></div>
        <div class="row"><div class="k">Number</div><div class="v mono" id="r1_number">—</div></div>
        <div class="row"><div class="k">RFID UID</div><div class="v mono" id="r1_tag">—</div></div>
      </div>
      <div class="form">
        <input id="r1_name_in" placeholder="Owner name"/>
        <input id="r1_num_in" placeholder="Owner number (11 digits)" inputmode="numeric" maxlength="11"/>
      </div>
      <div class="btnbar">
        <button class="soft" onclick="saveOwner(1)">Save Owner</button>
        <button onclick="toggle(1)">Web Toggle</button>
        <button class="danger" onclick="unlock(1)">Kill Unlock</button>
        <button onclick="clearRack(1)">Clear Session</button>
      </div>
    </div>

    <div class="card">
      <div class="cardHead">
        <h2>Rack 2</h2>
        <div class="badges">
          <span class="badge" id="r2_lock_badge">—</span>
          <span class="badge" id="r2_bike_badge">—</span>
        </div>
      </div>
      <div class="kv">
        <div class="row"><div class="k">Owner</div><div class="v mono" id="r2_owner">—</div></div>
        <div class="row"><div class="k">Number</div><div class="v mono" id="r2_number">—</div></div>
        <div class="row"><div class="k">RFID UID</div><div class="v mono" id="r2_tag">—</div></div>
      </div>
      <div class="form">
        <input id="r2_name_in" placeholder="Owner name"/>
        <input id="r2_num_in" placeholder="Owner number (11 digits)" inputmode="numeric" maxlength="11"/>
      </div>
      <div class="btnbar">
        <button class="soft" onclick="saveOwner(2)">Save Owner</button>
        <button onclick="toggle(2)">Web Toggle</button>
        <button class="danger" onclick="unlock(2)">Kill Unlock</button>
        <button onclick="clearRack(2)">Clear Session</button>
      </div>
    </div>
  </div>

  <div class="card" style="margin-top:14px">
    <div class="cardHead">
      <h2>System</h2>
      <div class="badges">
        <span class="badge warn" id="last_update">—</span>
      </div>
    </div>

    <div class="row">
      <div class="k">Last Event</div>
      <div class="v" id="evt">—</div>
    </div>

    <div class="logBox">
      <div class="logHead">
        <div>
          <div class="logTitle">Activity Log</div>
          <div class="logMeta" id="log_meta">—</div>
        </div>
        <div class="logMeta">Newest first</div>
      </div>
      <div class="logList" id="logs">
        <div class="logRow"><div class="ts">—</div><div class="msg">Loading logs…</div></div>
      </div>
    </div>

    <details>
      <summary>Raw Status JSON</summary>
      <pre id="raw">{}</pre>
    </details>
  </div>
</div>

<script>
function bindPhoneLimit(id){
  const el = document.getElementById(id);
  el.addEventListener('input', () => {
    el.value = el.value.replace(/\\D/g,'').slice(0,11);
  });
}
bindPhoneLimit('r1_num_in');
bindPhoneLimit('r2_num_in');

function setDot(kind){
  const dot = document.getElementById('dot');
  dot.style.background = (kind==='ok') ? 'var(--good)'
                     : (kind==='bad') ? 'var(--bad)'
                     : 'var(--warn)';
}

function setBadge(el, kind, text){
  el.textContent = text;
  el.classList.remove('good','bad','warn');
  if(kind) el.classList.add(kind);
}

function escHtml(s){
  return String(s)
    .replace(/&/g,'&amp;')
    .replace(/</g,'&lt;')
    .replace(/>/g,'&gt;')
    .replace(/"/g,'&quot;')
    .replace(/'/g,'&#039;');
}

async function api(path, opts){
  return fetch(path, opts||{});
}

function nowClock(){
  const d = new Date();
  const hh = String(d.getHours()).padStart(2,'0');
  const mm = String(d.getMinutes()).padStart(2,'0');
  const ss = String(d.getSeconds()).padStart(2,'0');
  return `${hh}:${mm}:${ss}`;
}

async function refresh(){
  try{
    const r = await api('/status', {cache:'no-store'});
    const j = await r.json();

    const r1Lock = document.getElementById('r1_lock_badge');
    const r2Lock = document.getElementById('r2_lock_badge');
    const r1Bike = document.getElementById('r1_bike_badge');
    const r2Bike = document.getElementById('r2_bike_badge');

    setBadge(r1Lock, j.rack1.locked ? 'bad' : 'good', j.rack1.locked ? 'LOCKED' : 'UNLOCKED');
    setBadge(r2Lock, j.rack2.locked ? 'bad' : 'good', j.rack2.locked ? 'LOCKED' : 'UNLOCKED');

    setBadge(r1Bike, j.rack1.bike_present ? 'warn' : '', j.rack1.bike_present ? 'BIKE: YES' : 'BIKE: NO');
    setBadge(r2Bike, j.rack2.bike_present ? 'warn' : '', j.rack2.bike_present ? 'BIKE: YES' : 'BIKE: NO');

    document.getElementById('r1_owner').textContent = j.rack1.owner_name || '(none)';
    document.getElementById('r2_owner').textContent = j.rack2.owner_name || '(none)';
    document.getElementById('r1_number').textContent = j.rack1.owner_number || '(none)';
    document.getElementById('r2_number').textContent = j.rack2.owner_number || '(none)';
    document.getElementById('r1_tag').textContent = j.rack1.tag_uid || '—';
    document.getElementById('r2_tag').textContent = j.rack2.tag_uid || '—';

    const headline = document.getElementById('headline');
    if (j.rack1.locked || j.rack2.locked) { headline.textContent = 'Active • Locked rack(s)'; setDot('bad'); }
    else { headline.textContent = 'Ready • Both unlocked'; setDot('ok'); }

    document.getElementById('evt').textContent = j.last_event || '';
    setBadge(document.getElementById('last_update'), 'warn', 'Updated ' + nowClock());

    const logsEl = document.getElementById('logs');
    const logMeta = document.getElementById('log_meta');

    if (j.logs && Array.isArray(j.logs) && j.logs.length) {
      logMeta.textContent = `${j.logs.length} shown`;
      logsEl.innerHTML = j.logs.map(e => {
        const t = escHtml(e.t || '—');
        const m = escHtml(e.msg || '');
        return `<div class="logRow"><div class="ts">${t}</div><div class="msg">${m}</div></div>`;
      }).join('');
    } else {
      logMeta.textContent = 'No logs yet';
      logsEl.innerHTML = `<div class="logRow"><div class="ts">—</div><div class="msg">No logs yet.</div></div>`;
    }

    document.getElementById('raw').textContent = JSON.stringify(j, null, 2);
  } catch(e) {}
}

async function saveOwner(n){
  const name = document.getElementById(`r${n}_name_in`).value.trim();
  const num  = document.getElementById(`r${n}_num_in`).value.trim();
  const q = new URLSearchParams({rack:String(n), name, number:num});
  const r = await api('/setOwner?' + q.toString());
  alert(await r.text());
  refresh();
}

async function toggle(n){
  const r = await api('/toggle?rack=' + n, {method:'POST'});
  alert(await r.text());
  refresh();
}

async function unlock(n){
  if(!confirm('Override UNLOCK for Rack ' + n + '?')) return;
  const r = await api('/unlock?rack=' + n, {method:'POST'});
  alert(await r.text());
  refresh();
}

async function clearRack(n){
  if(!confirm('Clear session (name/number) for Rack ' + n + '?')) return;
  const r = await api('/clear?rack=' + n, {method:'POST'});
  alert(await r.text());
  refresh();
}

setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>/*
  MAIN ESP32-WROOM-32
  Barangay Bike Rack System (2 racks)

  - WiFi AP + web UI
  - LCD + keypad self-service (keypad on ESP32-C3 via UART)
  - RFID control (RC522 on right-side VSPI pins)
  - Limit switches for bike presence + tamper detection
  - SMS alerts via phone gateway
  - 2 linear actuators driven by 4 SPDT relays (2 per actuator)
…
)HTML";

// =====================================================
//                 Web Routes / JSON status
// =====================================================
String rackJson(int idx) {
  String tagStr = uidToString(racks[idx].tagUid, racks[idx].tagLen);

  String ownerName = racks[idx].ownerName; ownerName.replace("\\", "\\\\"); ownerName.replace("\"", "\\\"");
  String ownerNumber = racks[idx].ownerNumber; ownerNumber.replace("\\", "\\\\"); ownerNumber.replace("\"", "\\\"");

  String json = "{";
  json += "\"locked\":" + String(racks[idx].locked ? "true" : "false") + ",";
  json += "\"bike_present\":" + String(bikePresent[idx] ? "true" : "false") + ",";
  json += "\"owner_name\":\"" + ownerName + "\",";
  json += "\"owner_number\":\"" + ownerNumber + "\",";
  json += "\"tag_uid\":\"" + tagStr + "\"";
  json += "}";
  return json;
}

int getRackArg() {
  String s = server.arg("rack");
  if (s.length() == 0) return -1;
  int rack = s.toInt();
  if (rack < 1 || rack > 2) return -1;
  return rack;
}

void handleSetOwner() {
  if (!checkAuth()) return;

  int rack = getRackArg();
  if (rack == -1) { server.send(400, "text/plain", "Invalid rack"); return; }
  int idx = rack - 1;

  String name = server.arg("name");
  String number = server.arg("number");

  if (number.length() > 0) {
    if (!digitsOnly(number)) { server.send(400, "text/plain", "Number must be digits only"); return; }
    if (number.length() != 11) { server.send(400, "text/plain", "Number must be exactly 11 digits"); return; }
  }

  racks[idx].ownerName = name;
  racks[idx].ownerNumber = number;

  lastEvent = "Owner updated (Rack " + String(rack) + ")";
  addLog("WEB SET OWNER Rack " + String(rack) + " Name=" + name + " Number=" + maskNumber(number));
  server.send(200, "text/plain", "OK: Owner saved for Rack " + String(rack));
}

void handleToggle() {
  if (!checkAuth()) return;

  int rack = getRackArg();
  if (rack == -1) { server.send(400, "text/plain", "Invalid rack"); return; }
  int idx = rack - 1;

  toggleLock(idx);
  lastEvent = "WEB toggle (Rack " + String(rack) + ")";
  addLog(String("WEB TOGGLE Rack ") + rack + (racks[idx].locked ? " -> LOCK" : " -> UNLOCK"));

  server.send(200, "text/plain", String("OK: Rack ") + rack + (racks[idx].locked ? " LOCKED" : " UNLOCKED"));
}

void handleUnlock() {
  if (!checkAuth()) return;

  int rack = getRackArg();
  if (rack == -1) { server.send(400, "text/plain", "Invalid rack"); return; }
  int idx = rack - 1;

  setLocked(idx, false);
  lastEvent = "KILL UNLOCK (Rack " + String(rack) + ")";
  addLog("WEB KILL UNLOCK Rack " + String(rack));

  server.send(200, "text/plain", "OK: Rack unlocked by barangay override.");
}

void handleClear() {
  if (!checkAuth()) return;

  int rack = getRackArg();
  if (rack == -1) { server.send(400, "text/plain", "Invalid rack"); return; }
  int idx = rack - 1;

  racks[idx].ownerName = "";
  racks[idx].ownerNumber = "";
  racks[idx].locked = false;
  racks[idx].tamperLatched = false;
  // Do not move actuators on clear; just leave state unlocked.

  lastEvent = "Session cleared (Rack " + String(rack) + ")";
  addLog("WEB CLEAR Rack " + String(rack));
  server.send(200, "text/plain", "OK: Session cleared (name/number) for Rack " + String(rack));
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", HTTP_GET, []() {
    if (!checkAuth()) return;
    server.send(200, "text/html", FPSTR(INDEX_HTML));
  });

  server.on("/status", HTTP_GET, []() {
    if (!checkAuth()) return;

    String le = lastEvent;
    le.replace("\\", "\\\\");
    le.replace("\"", "\\\"");

    String json = "{";
    json += "\"rack1\":" + rackJson(0) + ",";
    json += "\"rack2\":" + rackJson(1) + ",";
    json += "\"last_event\":\"" + le + "\",";
    json += "\"logs\":" + logsJson();
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/setOwner", HTTP_GET, handleSetOwner);

  server.on("/toggle", HTTP_POST, handleToggle);
  server.on("/toggle", HTTP_GET,  handleToggle);

  server.on("/unlock", HTTP_POST, handleUnlock);
  server.on("/unlock", HTTP_GET,  handleUnlock);

  server.on("/clear", HTTP_POST, handleClear);
  server.on("/clear", HTTP_GET,  handleClear);

  server.begin();
  lastEvent = "Web server started";
  addLog("WEB Server started");
}

// =====================================================
//                         Setup / Loop
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // LCD init
  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();
  uiShowIdle();

  // Assign fixed tags
  racks[0].tagUid = RACK1_UID; racks[0].tagLen = UID_LEN;
  racks[1].tagUid = RACK2_UID; racks[1].tagLen = UID_LEN;

  // Switch pins: internal pull-up
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);

  bikePresent[0] = readBikePresent(SW1_PIN);
  bikePresent[1] = readBikePresent(SW2_PIN);
  lastBikePresent[0] = bikePresent[0];
  lastBikePresent[1] = bikePresent[1];

  // Actuator relay pins
  pinMode(R1_RELAY_A_PIN, OUTPUT);
  pinMode(R1_RELAY_B_PIN, OUTPUT);
  pinMode(R2_RELAY_A_PIN, OUTPUT);
  pinMode(R2_RELAY_B_PIN, OUTPUT);
  allStop();

  // Init motion states
  rackMotion[0].state = MOTION_IDLE;
  rackMotion[1].state = MOTION_IDLE;
  racks[0].locked = false;
  racks[1].locked = false;
  racks[0].tamperLatched = false;
  racks[1].tamperLatched = false;

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();

  // UART link from ESP32-C3 keypad
  KeypadSerial.begin(KEYPAD_BAUD, SERIAL_8N1, KEYPAD_UART_RX_PIN, -1);

  // RFID setup (boot-safe init)
  rfidSafeBootInit();

  // Web setup
  setupWeb();

  lastEvent = "System ready (Phone SMS Gateway)";
  addLog("BOOT System ready");
}

void loop() {
  server.handleClient();

  updateBikePresence();
  checkTamper();

  handleSelfServeKeypad();
  handleRFID();

  updateMotion();
  updateBuzzer();

  // If self-service in progress and bike disappears, reset UI
  if ((uiState != UI_IDLE) && uiSelectedRack != -1) {
    if (!bikePresent[uiSelectedRack]) {
      lcd2("Bike Removed", "Session reset");
      delay(2000);
      resetUi();
    }
  }
}
