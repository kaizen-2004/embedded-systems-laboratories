#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ==================== OLED SETUP ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==================== PIN DEFINITIONS ====================
#define ONE_WIRE_BUS  2   // DS18B20 data line
#define RELAY_PIN     8   // Relay control pin (most modules are active LOW)
#define SWITCH_PIN    3   // Toggle switch (to GND) : OFF=Cold big, ON=Hot big

#define RELAY_ON   LOW
#define RELAY_OFF  HIGH

// ==================== RGB LED (ADDED) ====================
// PWM pins on UNO: 3,5,6,9,10,11 (we use 5,6,9)
#define RGB_R_PIN  5
#define RGB_G_PIN  6
#define RGB_B_PIN  9

// Set to 1 if your RGB is COMMON ANODE (common pin to +5V)
// Set to 0 if COMMON CATHODE (common pin to GND)
#define RGB_COMMON_ANODE 0

// LED modes (use uint8_t so Arduino auto-prototypes never break)
static const uint8_t LED_IDLE       = 0; // green
static const uint8_t LED_COOLING    = 1; // blue
static const uint8_t LED_SAFETY     = 2; // solid red
static const uint8_t LED_FAULTBLINK = 3; // blinking red
static const uint8_t LED_HOTBLINK   = 4; // fast blinking red

static uint8_t ledMode = LED_IDLE;
static bool ledBlinkState = false;
static unsigned long ledNext = 0;

// ==================== DS18B20 SETUP ====================
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Your sensor addresses (locked roles)
// HOT  : 28216736F3C86CF3
// COLD : 28616636F31813AA
DeviceAddress HOT_ADDR  = { 0x28, 0x21, 0x67, 0x36, 0xF3, 0xC8, 0x6C, 0xF3 };
DeviceAddress COLD_ADDR = { 0x28, 0x61, 0x66, 0x36, 0xF3, 0x18, 0x13, 0xAA };

// Optional calibration offsets (adjust after side-by-side test)
const float HOT_OFFSET_C  = 0.0;
const float COLD_OFFSET_C = 0.0;

// ==================== CONTROL PARAMETERS ====================
float targetTemp = 30.0;   // "Threshold" (cold-side target)
float hysteresis = 2.0;    // band around target
float maxHotTemp = 60.0;   // safety cutoff (only if HOT sensor is valid)

// Sensor validity limits
float minValidTemp = -10.0;
float maxValidTemp = 125.0;

// DS18B20 resolution
const uint8_t TEMP_RES_BITS = 12; // 12-bit = 750ms conversion

// ==================== STATE ====================
enum TempMode : uint8_t {
  MODE_DUAL_REAL,     // hot + cold real
  MODE_FAILSAFE_OFF   // cannot run safely
};

TempMode mode = MODE_FAILSAFE_OFF;

float coldTempC = NAN;
float hotTempC  = NAN;
float deltaTC   = NAN;

bool peltierOn      = false;
bool safetyOverride = false;
bool coldFault      = false;
bool hotFault       = false;

bool showHotBig = false;   // from toggle switch

// ==================== Efficiency scoring (0–100%) ====================
// Rolling dT window (1 sample per second)
float effPct = NAN;
const uint8_t DT_WINDOW = 20;     // 20s window
float dtBuf[DT_WINDOW];
uint8_t dtIdx = 0;
uint8_t dtCount = 0;

// Timing
unsigned long lastTick = 0;
const unsigned long intervalMs = 1000;

// ==================== HELPERS ====================
static bool isValidRawTemp(float t, float minT, float maxT) {
  if (t == DEVICE_DISCONNECTED_C) return false;
  if (t == 85.0) return false;
  if (t < minT || t > maxT) return false;
  return true;
}

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ==================== RGB FUNCTIONS (ADDED) ====================
static void writeRgb(uint8_t r, uint8_t g, uint8_t b) {
#if RGB_COMMON_ANODE
  r = 255 - r;
  g = 255 - g;
  b = 255 - b;
#endif
  analogWrite(RGB_R_PIN, r);
  analogWrite(RGB_G_PIN, g);
  analogWrite(RGB_B_PIN, b);
}

static void setLedMode(uint8_t m) {
  if (m == ledMode) return;
  ledMode = m;
  ledBlinkState = false;
  ledNext = 0;
}

static void updateLed(unsigned long now) {
  bool faultNow = (coldFault || hotFault);
  bool hotOverNow = (mode == MODE_DUAL_REAL && !hotFault && !isnan(hotTempC) && (hotTempC > maxHotTemp));

  if (faultNow) setLedMode(LED_FAULTBLINK);
  else if (hotOverNow) setLedMode(LED_HOTBLINK);
  else if (safetyOverride) setLedMode(LED_SAFETY);
  else if (peltierOn) setLedMode(LED_COOLING);
  else setLedMode(LED_IDLE);

  switch (ledMode) {
    case LED_IDLE:
      writeRgb(0, 255, 0);   // green
      break;

    case LED_COOLING:
      writeRgb(0, 0, 255);   // blue
      break;

    case LED_SAFETY:
      writeRgb(255, 0, 0);   // solid red
      break;

    case LED_FAULTBLINK: {
      const unsigned long blinkMs = 250;
      if (ledNext == 0 || now >= ledNext) {
        ledBlinkState = !ledBlinkState;
        ledNext = now + blinkMs;
      }
      writeRgb(ledBlinkState ? 255 : 0, 0, 0);
    } break;

    case LED_HOTBLINK: {
      const unsigned long blinkMs = 120;
      if (ledNext == 0 || now >= ledNext) {
        ledBlinkState = !ledBlinkState;
        ledNext = now + blinkMs;
      }
      writeRgb(ledBlinkState ? 255 : 0, 0, 0);
    } break;
  }
}

// ==================== EFFICIENCY UPDATE ====================
void updateEfficiency() {
  if (mode != MODE_DUAL_REAL || isnan(deltaTC)) {
    effPct = NAN;
    dtCount = 0;
    return;
  }

  dtBuf[dtIdx] = deltaTC;
  dtIdx = (dtIdx + 1) % DT_WINDOW;
  if (dtCount < DT_WINDOW) dtCount++;

  float sum = 0.0f, mn = 9999.0f, mx = -9999.0f;
  for (uint8_t i = 0; i < dtCount; i++) {
    float v = dtBuf[i];
    sum += v;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  float avg = sum / dtCount;
  float noise = mx - mn;

  const float DT_GOOD   = 20.0f;
  const float NOISE_BAD = 3.0f;

  float base = clampf((avg / DT_GOOD) * 100.0f, 0.0f, 100.0f);
  float stabilityPenalty = clampf((noise / NOISE_BAD) * 30.0f, 0.0f, 30.0f);

  float trendPenalty = 0.0f;
  if (peltierOn && dtCount >= 6) {
    uint8_t idx5 = (dtIdx + DT_WINDOW - 6) % DT_WINDOW;
    float dT_5s_ago = dtBuf[idx5];
    float slope = (deltaTC - dT_5s_ago) / 5.0f;

    if (slope < -0.05f) {
      trendPenalty = clampf(((-slope - 0.05f) / 0.25f) * 40.0f, 0.0f, 40.0f);
    }
  }

  float score = base - stabilityPenalty - trendPenalty;
  effPct = clampf(score, 0.0f, 100.0f);
}

// ==================== TEMPERATURE UPDATE (ASYNC READ) ====================
void updateTemperatures() {
  float coldRaw = sensors.getTempC(COLD_ADDR);
  float hotRaw  = sensors.getTempC(HOT_ADDR);

  coldFault = !isValidRawTemp(coldRaw, minValidTemp, maxValidTemp);
  hotFault  = !isValidRawTemp(hotRaw,  minValidTemp, maxValidTemp);

  if (coldFault) {
    mode = MODE_FAILSAFE_OFF;
    coldTempC = NAN;
    hotTempC  = NAN;
    deltaTC   = NAN;
    updateEfficiency();
    return;
  }

  coldTempC = coldRaw + COLD_OFFSET_C;

  if (!hotFault) {
    mode = MODE_DUAL_REAL;
    hotTempC = hotRaw + HOT_OFFSET_C;
    deltaTC = hotTempC - coldTempC;
  } else {
    mode = MODE_FAILSAFE_OFF;
    hotTempC = NAN;
    deltaTC = NAN;
  }

  updateEfficiency();
}

// ==================== CONTROL LOGIC ====================
void controlPeltier() {
  safetyOverride = false;

  if (mode == MODE_FAILSAFE_OFF) safetyOverride = true;

  if (!safetyOverride && mode == MODE_DUAL_REAL && hotTempC > maxHotTemp) {
    safetyOverride = true;
  }

  if (safetyOverride) {
    peltierOn = false;
    digitalWrite(RELAY_PIN, RELAY_OFF);
    return;
  }

  if (!peltierOn && coldTempC > (targetTemp + hysteresis)) {
    peltierOn = true;
    digitalWrite(RELAY_PIN, RELAY_ON);
  } else if (peltierOn && coldTempC < (targetTemp - hysteresis)) {
    peltierOn = false;
    digitalWrite(RELAY_PIN, RELAY_OFF);
  }
}

// ==================== OLED UI ====================
void drawStatusBar() {
  display.fillRect(0, 0, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK);

  display.setCursor(2, 1);
  if (safetyOverride) {
    display.print(F("SAFETY OFF"));
  } else {
    display.print(F("PELTIER "));
    display.print(peltierOn ? F("ON") : F("OFF"));
  }

  display.setCursor(104, 1);
  if (coldFault || hotFault) {
    display.print(F("ERR"));
  } else if (mode == MODE_DUAL_REAL && hotTempC > maxHotTemp) {
    display.print(F("HOT"));
  }
}

void updateDisplay() {
  display.clearDisplay();
  drawStatusBar();
  display.setTextColor(SSD1306_WHITE);

  if (coldFault) {
    display.setTextSize(1);
    display.setCursor(0, 18);
    display.print(F("Cold sensor problem"));
    display.setCursor(0, 30);
    display.print(F("Peltier forced OFF"));
    display.display();
    return;
  }

  if (hotFault) {
    display.setTextSize(1);
    display.setCursor(0, 18);
    display.print(F("Hot sensor problem"));
    display.setCursor(0, 30);
    display.print(F("Peltier forced OFF"));
    display.display();
    return;
  }

  float mainTemp = showHotBig ? hotTempC : coldTempC;

  display.setTextSize(1);
  display.setCursor(0, 12);
  display.print(showHotBig ? F("Hot side") : F("Cold Side: "));

  display.setTextSize(3);
  display.setCursor(0, 22);
  display.print(mainTemp, 1);
  display.print(F("C"));

  display.setTextSize(1);
  display.setCursor(0, 48);
  if (showHotBig) {
    display.print(F("Cold Side: "));
    display.setCursor(90, 48);
    display.print(coldTempC, 1);
    display.print(F("C"));
  } else {
    display.print(F("Hot Side: "));
    display.setCursor(90, 48);
    display.print(hotTempC, 1);
    display.print(F("C"));
  }

  display.setCursor(0, 56);
  display.print(F("Target: "));
  display.setCursor(90, 56);
  display.print(targetTemp, 1);
  display.print(F("C"));

  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);
  display.drawFastHLine(0, 44, SCREEN_WIDTH, SSD1306_WHITE);

  display.display();
}

// ==================== SERIAL DEBUG ====================
void printSerial() {
  Serial.print(F("Cold=")); Serial.print(coldTempC, 1); Serial.print(F("C "));
  Serial.print(F("Hot="));  Serial.print(hotTempC, 1);  Serial.print(F("C "));
  Serial.print(F("dT="));   if (!isnan(deltaTC)) Serial.print(deltaTC, 1); else Serial.print(F("NaN"));
  Serial.print(F(" Eff=")); if (!isnan(effPct))  Serial.print((int)(effPct + 0.5f)); else Serial.print(F("NaN"));
  Serial.print(F("% "));
  Serial.print(F("Peltier=")); Serial.print(peltierOn ? F("ON") : F("OFF"));
  Serial.print(F(" Safety=")); Serial.print(safetyOverride ? F("ON") : F("OFF"));
  Serial.print(F(" Big="));    Serial.println(showHotBig ? F("HOT") : F("COLD"));
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // RGB init
  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  writeRgb(0, 0, 0);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Peltier Cooler"));
  display.println(F("Init sensors..."));
  display.display();

  // DS18B20 init
  sensors.begin();
  sensors.setResolution(COLD_ADDR, TEMP_RES_BITS);
  sensors.setResolution(HOT_ADDR,  TEMP_RES_BITS);

  // Initial blocking conversion so first screen is valid
  sensors.setWaitForConversion(true);
  sensors.requestTemperatures();

  // First update
  showHotBig = (digitalRead(SWITCH_PIN) == LOW); // ON=LOW
  updateTemperatures();
  controlPeltier();
  updateDisplay();
  printSerial();

  // Show RGB immediately
  updateLed(millis());

  // Switch to async conversions
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  lastTick = millis();
}

// ==================== LOOP ====================
void loop() {
  unsigned long now = millis();

  // Keep RGB blinking responsive even between 1s ticks
  updateLed(now);

  if (now - lastTick >= intervalMs) {
    lastTick = now;

    showHotBig = (digitalRead(SWITCH_PIN) == LOW);

    updateTemperatures();
    controlPeltier();
    updateDisplay();
    printSerial();

    sensors.requestTemperatures();
  }
}
