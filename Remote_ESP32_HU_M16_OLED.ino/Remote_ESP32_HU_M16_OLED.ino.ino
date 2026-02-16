/*
Remote_ESP32_HU_M16_OLED.ino

Patched remote controller:
- Forces I2C pins and 400 kHz clock (Wire.begin(21,22); Wire.setClock(400000))
- Uses detected 7-bit OLED address 0x3C (8-bit 0x78)
- Keeps send feedback on Serial and OLED
- Keeps joystick center calibration, throttle safety for arming, and ESP-NOW send/CB

Wiring (I2C pins used here):
- SDA -> GPIO21
- SCL -> GPIO22
- VCC -> 3.3V
- GND -> GND
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2lib.h>
#include <Preferences.h>

#include "rc_message.h" // shared packed struct
// create U8g2 object (global)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ---------- Config / Pins ----------
#define JOY_X_PIN      35   // ADC1_CH0 (input-only)
#define JOY_Y_PIN      34   // ADC1_CH3 (input-only)
#define JOY_BUTTON_PIN 32   // digital input (pushbutton on joystick)
#define THROTTLE_PIN   39   // optional analog throttle (set to -1 if unused)
#define YAW_PIN        36   // optional analog yaw control (set to -1 if unused)

#define USE_THROTTLE_PIN  true // set false if no throttle pot
#define USE_YAW_PIN       true // set false if no yaw pot

// Logarithmic mapping strengths (tune to taste)
// k == 0 => linear (no change). Larger k => stronger curve (less sensitive near center).
const float LOG_K_JOY = 3.0f;   // joystick X/Y curve strength
const float LOG_K_YAW = 4.0f;   // yaw pot curve strength
const float LOG_K_THR = 6.0f;   // throttle curve strength (applied to signed tnorm)

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
// Using 7-bit address discovered by the scanner
#define OLED_ADDR     0x3C

// I2C pins (ESP32 typical)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// ADC settings
const int ADC_BITS = 12;        // 0..4095
const int ADC_MAX = (1 << ADC_BITS) - 1;

// joystick mapping
const float MAX_ANGLE_DEG = 30.0f;     // joystick full deflection => +/- this angle
const float MAX_YAW_RATE_DPS = 120.0f; // yaw pot full deflection => +/- this yaw rate
const float DEADZONE = 0.08f;         // joystick deadzone around center (normalized)

// Safety: require low throttle to arm
const float ARM_THROTTLE_MAX = 0.10f;

// ESP-NOW send interval
const unsigned long SEND_INTERVAL_MS = 25; // ~40Hz

// ---------- Globals ----------
rc_message_t rc_msg = {0,0,0,0,0,0};

unsigned long lastSendMs = 0;
unsigned long lastDisplayMs = 0;
unsigned long seqCounter = 0;

int buttonState = HIGH;
int lastButtonState = HIGH;
unsigned long lastDebounceMs = 0;
const unsigned long debounceDelay = 50;
bool armed = true;

int centerX = ADC_MAX / 2;
int centerY = ADC_MAX / 2;
int centerThr = ADC_MAX / 2;
int centerYaw = ADC_MAX / 2;
bool calibratedCenter = false;

// Preferences for persistent calibration
Preferences prefs;

// send feedback
volatile bool lastSendSuccess = false;      // set from send callback
volatile unsigned long lastSendCallbackMs = 0;
unsigned long sendCount = 0;
unsigned long sendFailCount = 0;
int lastEspNowReturn = 0;

// broadcast address (default)
uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ---------- Helpers ----------
float normalizeAxis(int raw, int center) {
  float v = (float)(raw - center) / (float)center;
  v = constrain(v, -1.0f, 1.0f);
  if (fabs(v) < DEADZONE) return 0.0f;
  return v;
}

int readAdcRaw(int pin) {
  if (pin < 0) return ADC_MAX/2;
  return analogRead(pin);
}

void saveCalibration() {
  prefs.putBool("calibrated", true);
  prefs.putInt("cX", centerX);
  prefs.putInt("cY", centerY);
  prefs.putInt("cT", centerThr);
  prefs.putInt("cYaw", centerYaw);
  Serial.println("Calibration saved to preferences.");
}

void calibrateJoystickCenter(unsigned long ms = 800) {
  Serial.println("Calibrating joystick center -- keep joystick released (not pressed)...");
  unsigned long t0 = millis();
  unsigned long count = 0;
  long sumX = 0, sumY = 0;
  long sumYaw = 0;
  while (millis() - t0 < ms) {
    sumX += readAdcRaw(JOY_X_PIN);
    sumY += readAdcRaw(JOY_Y_PIN);
    
    if (USE_YAW_PIN) sumYaw += readAdcRaw(YAW_PIN);
    count++;
    delay(6);
  }
  if (count > 0) {
    centerX = (int)(sumX / count);
    centerY = (int)(sumY / count);
    if (USE_THROTTLE_PIN) centerThr = 4095;
    if (USE_YAW_PIN) centerYaw = (int)(sumYaw / count);
  }
  calibratedCenter = true;
  Serial.print("Center X: "); Serial.print(centerX);
  Serial.print("  Center Y: "); Serial.println(centerY);
  Serial.print("  Center Thr: "); Serial.println(centerThr);
  Serial.print("  Center Yaw: "); Serial.println(centerYaw);

  // persist calibration
  saveCalibration();
}

/*
  logScale(x, k) - sign-preserving logarithmic mapping for x in [-1..1]
  k >= 0 controls curve strength:
    k == 0 => identity (linear)
    k > 0  => output = sign(x) * log(1 + k * |x|) / log(1 + k)
  This maps |x|==1 -> 1, |x|==0 -> 0, and preserves sign.
*/
float logScale(float x, float k) {
  if (k <= 0.0f) return x; // no-op linear
  float s = (x >= 0.0f) ? 1.0f : -1.0f;
  float a = fabs(x);
  float denom = log(1.0f + k);
  if (denom == 0.0f) return x;
  float num = log(1.0f + k * a);
  return s * (num / denom);
}

// ---------- ESP-NOW send callback (modern signature) ----------
void onSendCb(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  lastSendCallbackMs = millis();
  lastSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(10);

  // ADC config (ESP32)
  analogReadResolution(ADC_BITS);

  pinMode(JOY_BUTTON_PIN, INPUT_PULLUP);

  // Explicitly initialize I2C pins and set 400kHz (scanner found 0x3C at 400kHz)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  u8g2.begin();

  // init preferences
  prefs.begin("rc", false);

  // If a saved calibration exists, load it. Otherwise perform calibration.
  if (prefs.getBool("calibrated", false)) {
    centerX = prefs.getInt("cX", ADC_MAX/2);
    centerY = prefs.getInt("cY", ADC_MAX/2);
    centerThr = prefs.getInt("cT", ADC_MAX/2);
    centerYaw = prefs.getInt("cYaw", ADC_MAX/2);
    calibratedCenter = true;
    Serial.println("Loaded saved joystick calibration.");
    Serial.print("Center X: "); Serial.print(centerX);
    Serial.print("  Center Y: "); Serial.println(centerY);
    Serial.print("  Center Thr: "); Serial.println(centerThr);
    Serial.print("  Center Yaw: "); Serial.println(centerYaw);
  } else {
    // If user holds button low at boot, force calibration; otherwise do default calibration once.
    Serial.println("No saved calibration. Hold joystick button while powering to force recalibration.");
    delay(50); // small delay so user can still hold the button
    if (digitalRead(JOY_BUTTON_PIN) == LOW) {
      Serial.println("Button held -> forcing recalibration now...");
      calibrateJoystickCenter(1200);
    } else {
      calibrateJoystickCenter(900);
    }
  }

  // Print this remote's MAC so you can add as peer on flight controller if needed
  WiFi.mode(WIFI_STA);
  delay(50);
  Serial.print("Remote MAC: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW (retry once if fails)
  esp_err_t r = esp_now_init();
  if (r != ESP_OK) {
    Serial.print("esp_now_init failed (first attempt): ");
    Serial.println((int)r);
    delay(200);
    r = esp_now_init();
  }
  if (r != ESP_OK) {
    Serial.print("ESP-NOW init failed: ");
    Serial.println((int)r);
  } else {
    Serial.println("ESP-NOW initialized");
  }

  // register send callback and (optionally) add broadcast peer
  esp_now_register_send_cb(onSendCb);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_err_t addres = esp_now_add_peer(&peerInfo);
  (void)addres;

  // initial rc message
  rc_msg.seq = 0;
  rc_msg.arm = 0;
  rc_msg.throttle = 0.0f;

  Serial.println("Remote ready. Use joystick and button to control. See OLED for feedback (if present).");
}

// ---------- Main loop ----------
void loop() {
  unsigned long now = millis();

  // Read inputs
  int rawX = readAdcRaw(JOY_X_PIN);
  int rawY = readAdcRaw(JOY_Y_PIN);
  float nx = normalizeAxis(rawX, centerX);
  float ny = normalizeAxis(rawY, centerY);

  // Apply logarithmic mapping to joystick X/Y (preserves sign)
  float nx_log = logScale(nx, LOG_K_JOY);
  float ny_log = logScale(ny, LOG_K_JOY);

  float roll_set = nx_log * MAX_ANGLE_DEG;
  float pitch_set = -ny_log * MAX_ANGLE_DEG;

  // Throttle: handle asymmetric spans relative to calibrated center
  float throttle = 0.0f;
  float tnorm = 0.0f; // normalized throttle (-1..+1), zero at center
  int rawT = ADC_MAX/2;
  if (USE_THROTTLE_PIN) {
    int rawT = readAdcRaw(THROTTLE_PIN);
  
  // Map the full physical range of the stick to 0.0 - 1.0
  // Note: You might need to swap 4095 and 0 depending on your wiring
  float t_mapped = map(rawT, 4095, 0, 0, 1000) / 1000.0f; 
  
  // Safety deadzone at the very bottom
  if (t_mapped < DEADZONE) t_mapped = 0.0f;
  
  throttle = constrain(t_mapped, 0.0f, 1.0f);
  }

  // Yaw: handle asymmetric spans relative to calibrated center (maps to -1..+1)
  float yaw_rate = 0.0f;
  float nyaw = 0.0f;
  int rawYaw = ADC_MAX/2;
  if (USE_YAW_PIN) {
    rawYaw = readAdcRaw(YAW_PIN);

    // compute actual span on each side of center (use calibrated centerYaw)
    float spanPos = (float)(ADC_MAX - centerYaw); // span for values > center
    float spanNeg = (float)centerYaw;             // span for values < center

    if (rawYaw >= centerYaw) {
      // positive side
      if (spanPos <= 0.0f) nyaw = 0.0f;
      else nyaw = ((float)rawYaw - (float)centerYaw) / spanPos;
    } else {
      // negative side
      if (spanNeg <= 0.0f) nyaw = 0.0f;
      else nyaw = ((float)rawYaw - (float)centerYaw) / spanNeg;
    }

    // clamp and deadzone
    nyaw = constrain(nyaw, -1.0f, 1.0f);
    if (fabs(nyaw) < DEADZONE) nyaw = 0.0f;

    // Apply logarithmic mapping to yaw
    float nyaw_log = logScale(nyaw, LOG_K_YAW);

    // convert to desired units
    yaw_rate = nyaw_log * MAX_YAW_RATE_DPS;
  }


  // Prepare message
  rc_msg.roll_deg = roll_set;
  rc_msg.pitch_deg = pitch_set;
  rc_msg.yaw_rate_dps = yaw_rate;
  rc_msg.throttle = throttle;
  rc_msg.seq = ++seqCounter;

  // Send periodically
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    esp_err_t res = esp_now_send(broadcastAddress, (uint8_t *)&rc_msg, sizeof(rc_msg));
    lastEspNowReturn = (int)res;
    sendCount++;
    if (res != ESP_OK) {
      sendFailCount++;
      Serial.print("esp_now_send returned error: ");
      Serial.println((int)res);
    }
  }

  // Update OLED and Serial with feedback at ~20Hz
  if (now - lastDisplayMs >= 50) {
    lastDisplayMs = now;

    // Determine last send status for display
    const char *sendStatus = "WAIT";
    if (lastSendCallbackMs != 0 && (now - lastSendCallbackMs) < 1000) {
      sendStatus = lastSendSuccess ? "OK" : "FAIL";
    } else {
      if (lastEspNowReturn != ESP_OK) sendStatus = "ERR";
    }

    // Serial brief status
    Serial.print("S#"); Serial.print(sendCount);
    Serial.print(" /Fails:"); Serial.print(sendFailCount);
    Serial.print("  SendStatus:"); Serial.print(sendStatus);
    Serial.print("  seq:"); Serial.print(rc_msg.seq);
    Serial.print("  thr:"); Serial.print(rc_msg.throttle,3);
    Serial.print("  roll:"); Serial.print(rc_msg.roll_deg,2);
    Serial.print("  pitch:"); Serial.print(rc_msg.pitch_deg,2);
    Serial.print("  yaw:"); Serial.println(rc_msg.yaw_rate_dps,1);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    char buf[64];
    snprintf(buf, sizeof(buf), "ARM: %s", armed ? "YES":"NO");
    u8g2.drawStr(0,10, buf);
    snprintf(buf, sizeof(buf), "S#: %lu F:%lu", sendCount, sendFailCount);
    u8g2.drawStr(0,22, buf);
    snprintf(buf, sizeof(buf), "Send:%s", sendStatus);
    u8g2.drawStr(0,34, buf);
    snprintf(buf, sizeof(buf), "T:%.2f Y:%.1f", rc_msg.throttle, rc_msg.yaw_rate_dps);
    u8g2.drawStr(0,46, buf);
    snprintf(buf, sizeof(buf), "R:%.1f P:%.1f", rc_msg.roll_deg, rc_msg.pitch_deg);
    u8g2.drawStr(0,58, buf);
    u8g2.sendBuffer();
  }

  delay(4);
}