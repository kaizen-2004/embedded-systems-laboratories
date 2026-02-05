#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =========================
// ESP32-C3 I2C PINS (keep same wiring concept: shared I2C bus)
// =========================
static constexpr uint8_t I2C_SDA_PIN = 10;   // your SDA wire goes here
static constexpr uint8_t I2C_SCL_PIN = 9;    // your SCL wire goes here
static constexpr uint32_t I2C_FREQ_HZ = 400000; // optional

// =========================
// OLED CONFIG
// =========================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET   -1
#define OLED_ADDR    0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =========================
// MPU6050 CONFIG
// =========================
#define MPU_ADDR 0x68  // AD0 low = 0x68, AD0 high = 0x69

// Raw sensor variables
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Angle estimation
float angle = 0.0f;       // filtered angle
float gyroOffset = 0.0f;  // gyro bias around X axis

// Complementary filter parameter
const float alpha = 0.98f;

// Time tracking
unsigned long lastTime = 0;

// =========================
// FUNCTION DECLARATIONS
// =========================
static void initMPU6050();
static bool readMPU6050();
static float computeAngle(float dt);
static void updateOLED(float angleVal);

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(200);

  // I2C init (ESP32-C3 requires explicit SDA/SCL pins)
  if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN)) {
    Serial.println("ERROR: Wire.begin failed (check SDA/SCL pins)");
    while (true) delay(1000);
  }
  Wire.setClock(I2C_FREQ_HZ);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (true) delay(1000);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Boom Boom Balanci");
  display.setCursor(0, 10);
  display.println("Init MPU6050...");
  display.display();

  // Initialize MPU6050
  initMPU6050();
  delay(200);

  // Give some time to settle and get initial angle
  lastTime = millis();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("MPU6050 Ready!");
  display.display();
  delay(500);
}

// =========================
// MAIN LOOP
// =========================
void loop() {
  const unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  lastTime = now;

  // 1) Read IMU
  if (!readMPU6050()) {
    // I2C read failed; skip this loop iteration
    return;
  }

  // 2) Compute angle with complementary filter
  angle = computeAngle(dt);

  // 3) Show on OLED
  updateOLED(angle);

  // Debug
  Serial.print("Angle: ");
  Serial.println(angle, 2);

  delay(10);  // loop ~100 Hz
}

// =========================
// MPU6050 FUNCTIONS
// =========================
static void initMPU6050() {
  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0x00);   // wake up (clear sleep bit)
  Wire.endTransmission();
  delay(100);

  // Set gyro ±250 deg/s
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);   // GYRO_CONFIG
  Wire.write(0x00);   // ±250 deg/s
  Wire.endTransmission();

  // Set accel ±2g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);   // ACCEL_CONFIG
  Wire.write(0x00);   // ±2g
  Wire.endTransmission();

  // Simple gyro offset calibration (keep sensor still)
  const int N = 1000;
  long gyroSum = 0;

  Serial.println("Calibrating gyro offset, keep still...");
  for (int i = 0; i < N; i++) {
    if (readMPU6050()) {
      gyroSum += gx; // assume rotation around X for pitch
    }
    delay(2);
  }

  gyroOffset = (float)gyroSum / (float)N;
  Serial.print("Gyro offset (raw gx): ");
  Serial.println(gyroOffset, 2);
}

// Read raw accel and gyro data
static bool readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting register: ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;

  const uint8_t got = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (got != 14) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  (void)((Wire.read() << 8) | Wire.read()); // temperature (unused)
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

// Compute tilt angle (pitch) and apply complementary filter
static float computeAngle(float dt) {
  // Convert raw to physical values (±2g -> 16384 LSB/g)
  const float ax_g = ax / 16384.0f;
  const float ay_g = ay / 16384.0f;
  const float az_g = az / 16384.0f;

  // Gyro in deg/s (±250 dps -> 131 LSB/(deg/s))
  const float gx_dps = (gx - gyroOffset) / 131.0f;

  // Accelerometer-based angle (pitch) in degrees
  // Using X and Z axes; adjust if your orientation is different
  const float accelAngle =
      atan2f(ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / PI;

  // Integrate gyro to get gyro-based angle
  const float gyroAngle = angle + gx_dps * dt;

  // Complementary filter
  return alpha * gyroAngle + (1.0f - alpha) * accelAngle;
}

// =========================
// OLED DISPLAY
// =========================
static void updateOLED(float angleVal) {
  display.clearDisplay();
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("Angle: ");
  display.print(angleVal, 1);
  display.print((char)247); // degree symbol

  display.setCursor(0, 10);
  display.print("Boom Boom Balanci");

  display.setCursor(0, 20);
  display.print("Tilt to see change");

  display.display();
}
