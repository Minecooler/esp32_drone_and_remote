/* DoublePID_FlightController.ino
Updated for ESC Range: 920us - 2120us
*/

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <L3G.h>
#include <LSM303.h>
#include <Wire.h>
#include <SensorFusion.h>

// ----- CONFIG -----
const int MOTOR_PIN_FL = 33; // Front-Left
const int MOTOR_PIN_FR = 25; // Front-Right
const int MOTOR_PIN_BL = 32; // Back-Left
const int MOTOR_PIN_BR = 4;  // Back-Right

// ledc PWM settings
const int PWM_FREQ = 50;     // 50Hz -> 20ms period
const int PWM_RES = 16;      // 16-bit resolution
const int PWM_MAX = (1 << PWM_RES) - 1;

// YOUR SPECIFIC ESC LIMITS 
const int ESC_MIN_US = 920;  
const int ESC_MAX_US = 2120; 


// Safety/timeouts
const unsigned long RC_TIMEOUT_MS = 200;
const bool REQUIRE_ARM = true;

// ----- ESC Calibration detection settings -----
// These constants define the signals required to enter calibration mode.
const float THR_SIG_LOW  = 0.15f; // Stick is below 5%
const float THR_SIG_HIGH = 0.85f; // Stick is above 95%
const unsigned long CAL_HIGH_DURATION_MS = 4000UL; // How long to hold the high pulse [cite: 66]
const unsigned long CAL_WAIT_TIMEOUT_MS = 10000UL;  // Timeout for the handshake [cite: 66]

// ----- Sensors & fusion -----
SF fusion;
L3G gyro;
LSM303 compass;
const float GYRO_SCALE_TO_DPS = 1.0f; // Adjust based on your L3G settings

// ----- Calibration storage -----
struct CalData {
  uint32_t magic;
  float gyroBias[3];
  float accelOffset[3];
  float accelScale[3];
  float magOffset[3];
  float magScale[3];
};

const uint32_t CAL_MAGIC = 0xA5A5A5A5;
CalData cal;
Preferences prefs;

// ----- RC message via ESP-NOW -----
typedef struct __attribute__((packed)) {
  float roll_deg;
  float pitch_deg;
  float yaw_rate_dps;
  float throttle; // 0.0 .. 1.0
  uint32_t seq;
  uint8_t arm;
} rc_message_t;

volatile rc_message_t rc_in = {0};
volatile unsigned long rc_last_ms = 0;
volatile bool rc_new = false;
portMUX_TYPE rcMux = portMUX_INITIALIZER_UNLOCKED;

// ----- PID Structure -----
struct PID {
  float kp, ki, kd;
  float integrator, prev_error, out_min, out_max;
  float integrator_min, integrator_max;
  float derivative_filter_tau, d_filtered;

  void init(float _kp=0, float _ki=0, float _kd=0) {
    kp=_kp; ki=_ki; kd=_kd;
    integrator=0; prev_error=0;
    out_min=-1e6; out_max=1e6;
    integrator_min=-1e6; integrator_max=1e6;
    derivative_filter_tau = 0.02f;
    d_filtered = 0;
  }

  float update(float setpoint, float measurement, float dt) {
    if (dt <= 0) return 0;
    float err = setpoint - measurement;
    integrator += err * ki * dt;
    integrator = constrain(integrator, integrator_min, integrator_max);
    float deriv = (err - prev_error) / dt;
    float alpha = dt / (derivative_filter_tau + dt);
    d_filtered = d_filtered + alpha * (deriv - d_filtered);
    prev_error = err;
    float out = kp * err + integrator + kd * d_filtered;
    return constrain(out, out_min, out_max);
  }

  void reset() { integrator = 0; prev_error = 0; d_filtered = 0; }
};

PID rollAnglePID, pitchAnglePID; 
PID rollRatePID, pitchRatePID, yawRatePID;
const float MAX_DESIRED_RATE = 200.0f;

// ----- PWM Helpers -----
uint32_t microsToDuty(int us) {
  // Equivalent to: (us * 65535) / 20000
  // We use 65535ULL to ensure the multiplication doesn't overflow a 32-bit integer
  return (uint32_t)((us * 65535ULL) / 20000ULL);
}

void escAttach(int pin) {
  ledcAttach(pin, PWM_FREQ, PWM_RES);
}

void escWriteUs(int pin, int us) {
  us = constrain(us, ESC_MIN_US, ESC_MAX_US);
  ledcWrite(pin, microsToDuty(us));
}

const float THR_MIN_NORM = 18.0f / 255.0f; 
int throttleToUs(float thr_norm) {
  thr_norm = constrain(thr_norm, 0.0f, 1.0f);
  if (thr_norm <= THR_MIN_NORM) return ESC_MIN_US;
  float scaled = (thr_norm - THR_MIN_NORM) / (1.0f - THR_MIN_NORM);
  return (int)round(ESC_MIN_US + scaled * (ESC_MAX_US - ESC_MIN_US));
}

// ----- Motor Mixing (Quad-X) -----
void mixAndWriteMotors(int throttle_us, float roll_u, float pitch_u, float yaw_u) {
  // Front-Left: CW | Front-Right: CCW | Back-Left: CCW | Back-Right: CW
  int mFL = throttle_us + (int)round(pitch_u) + (int)round(roll_u) - (int)round(yaw_u);
  int mFR = throttle_us + (int)round(pitch_u) - (int)round(roll_u) + (int)round(yaw_u);
  int mBL = throttle_us - (int)round(pitch_u) + (int)round(roll_u) + (int)round(yaw_u);
  int mBR = throttle_us - (int)round(pitch_u) - (int)round(roll_u) - (int)round(yaw_u);

  escWriteUs(MOTOR_PIN_FL, mFL);
  escWriteUs(MOTOR_PIN_FR, mFR);
  escWriteUs(MOTOR_PIN_BL, mBL);
  escWriteUs(MOTOR_PIN_BR, mBR);
}

// ----- ESC Calibration & Failsafes -----
const uint8_t remoteMac[6] = { 0xE0, 0x8C, 0xFE, 0x5E, 0x25, 0x0C };
enum EscCalState { ESC_CAL_NOT_STARTED=0, ESC_CAL_WAIT_HIGH, ESC_CAL_RUNNING, ESC_CAL_DONE };
EscCalState escCalState = ESC_CAL_NOT_STARTED;
unsigned long escCalStartMs = 0, escCalWaitStartMs = 0;
bool escCalibrated = false;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(rc_message_t) || memcmp(info->src_addr, remoteMac, 6) != 0) return;
  portENTER_CRITICAL(&rcMux);
  memcpy((void*)&rc_in, incomingData, sizeof(rc_in));
  rc_last_ms = millis();
  rc_new = true;
  portEXIT_CRITICAL(&rcMux);
}
/* ---------------- Calibration routines & persistence ---------------- */

void saveCalibration() {
  prefs.begin("cal", false);
  prefs.putBytes("caldata", &cal, sizeof(cal));
  prefs.end();
  Serial.println("Calibration saved to Preferences.");
}

void loadCalibration() {
  prefs.begin("cal", true);
  if (prefs.isKey("caldata")) {
    size_t sz = prefs.getBytesLength("caldata");
    if (sz == sizeof(cal)) {
      prefs.getBytes("caldata", &cal, sizeof(cal));
      if (cal.magic == CAL_MAGIC) {
        Serial.println("Loaded calibration from storage.");
        prefs.end();
        return;
      }
    }
  }
  prefs.end();
  // set defaults if not present
  cal.magic = CAL_MAGIC;
  for (int i=0;i<3;i++) {
    cal.gyroBias[i]=0;
    cal.accelOffset[i]=0;
    cal.accelScale[i]=1;
    cal.magOffset[i]=0;
    cal.magScale[i]=1;
  }
  Serial.println("No calibration found: using defaults.");
}

void printCalibration() {
  Serial.println();
  Serial.println("CURRENT CALIBRATION:");
  Serial.print("gyroBias = { "); Serial.print(cal.gyroBias[0]); Serial.print(", ");
  Serial.print(cal.gyroBias[1]); Serial.print(", "); Serial.print(cal.gyroBias[2]); Serial.println(" }");

  Serial.print("accelOffset = { "); Serial.print(cal.accelOffset[0]); Serial.print(", ");
  Serial.print(cal.accelOffset[1]); Serial.print(", "); Serial.print(cal.accelOffset[2]); Serial.println(" }");

  Serial.print("accelScale = { "); Serial.print(cal.accelScale[0]); Serial.print(", ");
  Serial.print(cal.accelScale[1]); Serial.print(", "); Serial.print(cal.accelScale[2]); Serial.println(" }");

  Serial.print("magOffset = { "); Serial.print(cal.magOffset[0]); Serial.print(", ");
  Serial.print(cal.magOffset[1]); Serial.print(", "); Serial.print(cal.magOffset[2]); Serial.println(" }");

  Serial.print("magScale = { "); Serial.print(cal.magScale[0]); Serial.print(", ");
  Serial.print(cal.magScale[1]); Serial.print(", "); Serial.print(cal.magScale[2]); Serial.println(" }");
  Serial.println();
}

void waitForEnter() {
  Serial.println("(press Enter or any key in Serial Monitor)");
  while (!Serial.available()) { delay(10); }
  while (Serial.available()) Serial.read();
  delay(50);
}

void calibrateGyro() {
  const int N = 800;
  long sumX = 0, sumY = 0, sumZ = 0;
  Serial.println();
  Serial.println("*** Gyro calibration ***");
  Serial.println("Keep the board perfectly still on a stable surface.");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  for (int i = 0; i < N; ++i) {
    gyro.read();
    sumX += gyro.g.x;
    sumY += gyro.g.y;
    sumZ += gyro.g.z;
    delay(2);
  }

  cal.gyroBias[0] = (float)sumX / N;
  cal.gyroBias[1] = (float)sumY / N;
  cal.gyroBias[2] = (float)sumZ / N;

  Serial.println("Gyro bias (raw units):");
  Serial.print("  gx_bias = "); Serial.println(cal.gyroBias[0]);
  Serial.print("  gy_bias = "); Serial.println(cal.gyroBias[1]);
  Serial.print("  gz_bias = "); Serial.println(cal.gyroBias[2]);

  saveCalibration();
  Serial.println();
}

void calibrateAccel6Pos() {
  const char *positions[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  float readings[6][3];
  const int samples = 200;

  Serial.println();
  Serial.println("*** Accelerometer 6-position calibration ***");
  Serial.println("You will be prompted to place the board in each position.");
  Serial.println("Place board steady and press Enter to record that position.");
  Serial.println("Order: +X, -X, +Y, -Y, +Z, -Z");
  Serial.println();

  for (int p = 0; p < 6; ++p) {
    Serial.print("Place board in position ");
    Serial.print(positions[p]);
    Serial.println(" and press Enter...");
    waitForEnter();
    long sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < samples; ++i) {
      compass.read(); // updates compass.a.x etc
      sumX += compass.a.x;
      sumY += compass.a.y;
      sumZ += compass.a.z;
      delay(5);
    }
    readings[p][0] = (float)sumX / samples;
    readings[p][1] = (float)sumY / samples;
    readings[p][2] = (float)sumZ / samples;

    Serial.print("  recorded: ");
    Serial.print(readings[p][0]); Serial.print(", ");
    Serial.print(readings[p][1]); Serial.print(", ");
    Serial.println(readings[p][2]);
  }

  for (int axis = 0; axis < 3; ++axis) {
    float pos = readings[axis*2 + 0][axis];
    float neg = readings[axis*2 + 1][axis];
    cal.accelOffset[axis] = (pos + neg) / 2.0;
    float halfRange = (pos - neg) / 2.0;
    if (fabs(halfRange) < 1e-6) {
      cal.accelScale[axis] = 1.0;
    } else {
      cal.accelScale[axis] = 1.0 / halfRange;
    }
  }

  Serial.println("Accelerometer calibration results:");
  printCalibration();
  saveCalibration();
}

void calibrateMag() {
  const unsigned long collectMillis = 20000UL; // rotate for 20 seconds
  const int delayMs = 20;
  Serial.println();
  Serial.println("*** Magnetometer calibration ***");
  Serial.println("Rotate the board slowly in all axes for the next 20 seconds.");
  Serial.println("Press Enter to start.");
  waitForEnter();

  long t0 = millis();
  long tEnd = t0 + collectMillis;

  float minX =  1e6, minY =  1e6, minZ =  1e6;
  float maxX = -1e6, maxY = -1e6, maxZ = -1e6;

  while (millis() < tEnd) {
    compass.read();
    float mx = compass.m.x;
    float my = compass.m.y;
    float mz = compass.m.z;
    if (mx < minX) minX = mx;
    if (my < minY) minY = my;
    if (mz < minZ) minZ = mz;
    if (mx > maxX) maxX = mx;
    if (my > maxY) maxY = my;
    if (mz > maxZ) maxZ = mz;
    delay(delayMs);
  }

  cal.magOffset[0] = (maxX + minX) / 2.0;
  cal.magOffset[1] = (maxY + minY) / 2.0;
  cal.magOffset[2] = (maxZ + minZ) / 2.0;

  float rangeX = (maxX - minX) / 2.0;
  float rangeY = (maxY - minY) / 2.0;
  float rangeZ = (maxZ - minZ) / 2.0;
  float avg = (rangeX + rangeY + rangeZ) / 3.0;
  if (rangeX == 0) rangeX = 1;
  if (rangeY == 0) rangeY = 1;
  if (rangeZ == 0) rangeZ = 1;

  cal.magScale[0] = avg / rangeX;
  cal.magScale[1] = avg / rangeY;
  cal.magScale[2] = avg / rangeZ;

  Serial.println("Magnetometer calibration results:");
  printCalibration();
  saveCalibration();
}
bool isConnectedToUSB() {
  // On ESP32, Serial returns true if the USB-Serial bridge is active
  // and a terminal (like Arduino Serial Monitor) is open.
  if (Serial) {
    return true; 
  }
  return false;
}
/* -------------------- Setup & main loop -------------------- */
void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  loadCalibration();
  

  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onDataRecv);
  }
  else {
    Serial.println("ESP-NOW Init Failed");
  }

  escAttach(MOTOR_PIN_FL);
  escAttach(MOTOR_PIN_FR);
  escAttach(MOTOR_PIN_BL);
  escAttach(MOTOR_PIN_BR);
  
  /* Set to 920us initially
  escWriteUs(MOTOR_PIN_FL, ESC_MIN_US);
  escWriteUs(MOTOR_PIN_FR, ESC_MIN_US);
  escWriteUs(MOTOR_PIN_BL, ESC_MIN_US);
  escWriteUs(MOTOR_PIN_BR, ESC_MIN_US);
  */

  if (!gyro.init()) Serial.println("Gyro Fail"); else gyro.enableDefault();
  if (!compass.init()) Serial.println("Compass Fail"); else compass.enableDefault();

  // Initial PID Tuning (Placeholders)
  rollAnglePID.init(6.0f, 0.0f, 0.0f);
  pitchAnglePID.init(6.0f, 0.0f, 0.0f);
  rollRatePID.init(0.05f, 0.002f, 0.001f);
  pitchRatePID.init(0.05f, 0.002f, 0.001f);
  yawRatePID.init(0.02f, 0.001f, 0.0005f);
  
  // Constrain rate PID outputs to +/- 300us offset
  rollRatePID.out_min = -300; rollRatePID.out_max = 300;
  pitchRatePID.out_min = -300; pitchRatePID.out_max = 300;
  yawRatePID.out_min = -300; yawRatePID.out_max = 300;

  Serial.println("\n=== SYSTEM INITIALIZED ===");
  Serial.println("Type 's' to start flight loop...");
}

unsigned long lastLoop = 0;
bool startFlight = false;

void runFlightLoop(); // forward

void loop() {
  // service serial menu when not running main flight loop
  bool tethered = isConnectedToUSB();
  if (tethered && !startFlight) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'g') {
        calibrateGyro();
      } else if (c == 'a') {
        calibrateAccel6Pos();
      } else if (c == 'm') {
        calibrateMag();
      } else if (c == 'r') {
        calibrateGyro();
        calibrateAccel6Pos();
        calibrateMag();
      } else if (c == 'p') {
        printCalibration();
      } else if (c == 's') {
        Serial.println("Starting flight loop. Ensure it's safe (props off first).");
        startFlight = true;
      }
    }
    // light idle loop
    delay(10);
    return;
  }

  // If startFlight set, run the main control loop (non-returning)
  runFlightLoop();
}

/* ---------------- ESC calibration handler ---------------- */
// Helper: stop motors and reset integrators
void stopMotorsAndResetPIDs() {
  escWriteUs(MOTOR_PIN_FL, ESC_MIN_US);
  escWriteUs(MOTOR_PIN_FR, ESC_MIN_US);
  escWriteUs(MOTOR_PIN_BL, ESC_MIN_US);
  escWriteUs(MOTOR_PIN_BR, ESC_MIN_US);
  rollAnglePID.reset();
  pitchAnglePID.reset();
  rollRatePID.reset();
  pitchRatePID.reset();
  yawRatePID.reset();
}

// This function inspects rc_valid and throttle_in and advances the esc calibration state machine.
// When in ESC_CAL_RUNNING it actively writes calibration pulses to the motors.

void handleEscCalibration(bool rc_valid, float throttle_in) {
  const float tol = 0.05f; 
  if (escCalibrated || !rc_valid) return;

  switch (escCalState) {
    case ESC_CAL_NOT_STARTED:
      // Trigger if stick is LOWER than the threshold (0.15)
      if (throttle_in <= THR_SIG_LOW) { 
        escCalState = ESC_CAL_WAIT_HIGH;
        escCalWaitStartMs = millis();
        Serial.println("ESC cal: detected LOW. Now push stick to MAX (Top)...");
      }
      break;

    case ESC_CAL_WAIT_HIGH:
      if (millis() - escCalWaitStartMs > CAL_WAIT_TIMEOUT_MS) {
        Serial.println("ESC cal: Handshake timed out. Try again.");
        escCalState = ESC_CAL_NOT_STARTED;
      } 
      // Trigger if stick is HIGHER than the threshold (0.85)
      else if (throttle_in >= THR_SIG_HIGH) {
        escAttach(MOTOR_PIN_FL);
        escAttach(MOTOR_PIN_FR);
        escAttach(MOTOR_PIN_BL);
        escAttach(MOTOR_PIN_BR);

        escCalState = ESC_CAL_RUNNING;
        escCalStartMs = millis();
        Serial.println("ESC cal: HIGH detected! Sending 4s calibration pulse...");
      }
      break;

    case ESC_CAL_RUNNING:
      if (millis() - escCalStartMs < CAL_HIGH_DURATION_MS) {
        mixAndWriteMotors(ESC_MAX_US, 0, 0, 0); 
      } else {
        mixAndWriteMotors(ESC_MIN_US, 0, 0, 0);
        delay(2000); // Give the ESC time to store the setting
        escCalibrated = true;
        escCalState = ESC_CAL_DONE;
        Serial.println("ESC cal: DONE. Normal flight enabled.");
      }
      break;
  }
}

/* ---------------- Main flight loop (applies calibration) ---------------- */

void runFlightLoop() {
  // loop runs continuously until reset; it's the original control loop with calibration applied

  unsigned long now_us = micros();
  float dt = (now_us - lastLoop) / 1e6f;
  if (dt <= 0 || dt > 0.1f) dt = 0.005f;
  lastLoop = now_us;

  // Read sensors
  compass.read();
  gyro.read();

  // Apply gyro bias correction then scale
  float gx = ((float)gyro.g.x - cal.gyroBias[0]) * GYRO_SCALE_TO_DPS;
  float gy = ((float)gyro.g.y - cal.gyroBias[1]) * GYRO_SCALE_TO_DPS;
  float gz = ((float)gyro.g.z - cal.gyroBias[2]) * GYRO_SCALE_TO_DPS;

  // Apply accel offset & scale -> convert to g units
  float ax = (((float)compass.a.x - cal.accelOffset[0]) * cal.accelScale[0]);
  float ay = (((float)compass.a.y - cal.accelOffset[1]) * cal.accelScale[1]);
  float az = (((float)compass.a.z - cal.accelOffset[2]) * cal.accelScale[2]);

  // Apply mag corrections
  float mx = (((float)compass.m.x - cal.magOffset[0]) * cal.magScale[0]);
  float my = (((float)compass.m.y - cal.magOffset[1]) * cal.magScale[1]);
  float mz = (((float)compass.m.z - cal.magOffset[2]) * cal.magScale[2]);

  float deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);

  float roll_deg = fusion.getRoll();
  float pitch_deg = fusion.getPitch();
  float yaw_deg = fusion.getYaw();

  // RC availability & failsafe
  bool rc_valid = (millis() - rc_last_ms) < RC_TIMEOUT_MS;

  float throttle_in = 0.0f;
  float desired_roll = 0.0f;
  float desired_pitch = 0.0f;
  float desired_yaw_rate = 0.0f;

  if (rc_valid) {
    throttle_in = rc_in.throttle; // 0..1
    desired_roll = rc_in.roll_deg;
    desired_pitch = rc_in.pitch_deg;
    desired_yaw_rate = rc_in.yaw_rate_dps;
  }

  // Handle ESC calibration state machine before allowing normal control
  handleEscCalibration(rc_valid, throttle_in);

  // If calibration not completed, keep motors stopped (or writing calibration pulses if running)
  if (!escCalibrated) {
    // if calibration is actively running, handleEscCalibration already wrote pulses; just return
    if (escCalState == ESC_CAL_RUNNING) {
      // allow calibration pulses to be output for this loop iteration
      delay(1);
      return;
    }
    // otherwise ensure motors are stopped and integrators reset
    stopMotorsAndResetPIDs();
    delay(10);
    return;
  }

  bool armed = (!REQUIRE_ARM) || (rc_valid && rc_in.arm);

  if (!armed) {
    // stop motors and reset integrators
    escWriteUs(MOTOR_PIN_FL, ESC_MIN_US);
    escWriteUs(MOTOR_PIN_FR, ESC_MIN_US);
    escWriteUs(MOTOR_PIN_BL, ESC_MIN_US);
    escWriteUs(MOTOR_PIN_BR, ESC_MIN_US);
    rollAnglePID.reset();
    pitchAnglePID.reset();
    rollRatePID.reset();
    pitchRatePID.reset();
    yawRatePID.reset();
    delay(10);
    return;
  }

  // Outer PID: angle -> desired rate (deg/s)
  float desired_roll_rate = rollAnglePID.update(desired_roll, roll_deg, dt);
  float desired_pitch_rate = pitchAnglePID.update(desired_pitch, pitch_deg, dt);
  desired_roll_rate = constrain(desired_roll_rate, -MAX_DESIRED_RATE, MAX_DESIRED_RATE);
  desired_pitch_rate = constrain(desired_pitch_rate, -MAX_DESIRED_RATE, MAX_DESIRED_RATE);

  // Inner PID: rate -> motor offset (microseconds)
  // Ensure gyro axes sign matches PID sign; adjust sign if necessary for your board
  float roll_out_us  = rollRatePID.update(desired_roll_rate, gx, dt);
  float pitch_out_us = pitchRatePID.update(desired_pitch_rate, gy, dt);
  float yaw_out_us   = yawRatePID.update(desired_yaw_rate, gz, dt);

  int throttle_us = throttleToUs(throttle_in);

  mixAndWriteMotors(throttle_us, roll_out_us, pitch_out_us, yaw_out_us);

  // Debug print periodically
  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs > 2000) {
    lastPrintMs = millis();
    Serial.print("R:"); Serial.print(roll_deg,2);
    Serial.print(" P:"); Serial.print(pitch_deg,2);
    Serial.print(" targetR:"); Serial.print(desired_roll,2);
    Serial.print(" tr:"); Serial.print(desired_roll_rate,1);
    Serial.print(" ru:"); Serial.print(roll_out_us,1);
    Serial.print(" thr:"); Serial.print(throttle_in,2);
    Serial.print(" rc_valid:"); Serial.print(rc_valid);
    Serial.print(" escCalibrated:"); Serial.print(escCalibrated);
    Serial.println();
  }

  // loop continues
}