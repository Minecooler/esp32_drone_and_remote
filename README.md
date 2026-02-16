# ESP32 Drone & Remote Control System

[cite_start]This project is a high-performance quadcopter system utilizing **ESP-NOW** for low-latency communication[cite: 1, 115]. [cite_start]It features a dual-PID control architecture (Angle + Rate) and a custom remote controller with OLED telemetry and logarithmic stick mapping[cite: 1, 172, 173].

---

## 🛠 Hardware Configuration

### Remote Controller Wiring
[cite_start]The remote uses a 12-bit ADC resolution (0..4095) for precise stick input[cite: 6, 7].

| Component | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **OLED SDA** | [cite_start]GPIO 21 [cite: 1, 6] | I2C Data |
| **OLED SCL** | [cite_start]GPIO 22 [cite: 1, 6] | [cite_start]I2C Clock (400kHz) [cite: 36] |
| **Joystick X** | [cite_start]GPIO 35 [cite: 2] | Roll axis |
| **Joystick Y** | [cite_start]GPIO 34 [cite: 2] | Pitch axis |
| **Joystick Button** | [cite_start]GPIO 32 [cite: 2] | Digital Input |
| **Throttle Pot** | [cite_start]GPIO 39 [cite: 2] | Optional analog throttle |
| **Yaw Pot** | [cite_start]GPIO 36 [cite: 2] | Optional analog yaw |

### Drone (Flight Controller) Wiring
[cite_start]The motors follow a **Quad-X** configuration with specific PWM limits[cite: 87, 112].



| Motor | ESP32 Pin | Rotation |
| :--- | :--- | :--- |
| **Front-Left (FL)** | [cite_start]GPIO 33 [cite: 83] | [cite_start]CW [cite: 112] |
| **Front-Right (FR)** | [cite_start]GPIO 25 [cite: 84] | [cite_start]CCW [cite: 112] |
| **Back-Left (BL)** | [cite_start]GPIO 32 [cite: 84] | [cite_start]CCW [cite: 112] |
| **Back-Right (BR)** | [cite_start]GPIO 4 [cite: 84] | [cite_start]CW [cite: 112] |

---

## 🚀 Setup & Calibration

### 1. Remote Preparation
* [cite_start]**MAC Address:** Upon boot, the remote prints its MAC address to the Serial Monitor[cite: 44, 45]. [cite_start]Ensure this matches the `remoteMac` defined in the flight controller code[cite: 115].
* [cite_start]**Joystick Calibration:** On the first run, the system calibrates centers and saves them to preferences[cite: 21, 28, 37]. [cite_start]To force recalibration, hold the joystick button while powering on[cite: 42, 43].
* **Curve Tuning:** Stick sensitivity is refined via logarithmic mapping:
    * [cite_start]`LOG_K_JOY`: 3.0 (Roll/Pitch)[cite: 5, 54].
    * [cite_start]`LOG_K_YAW`: 4.0[cite: 5, 68].
    * [cite_start]`LOG_K_THR`: 6.0[cite: 5].

### 2. Drone Sensor Calibration
[cite_start]Access the Serial menu (115200 baud) and use the following commands[cite: 177]:
* [cite_start]**`g`**: Calibrate Gyro (Keep the board perfectly still)[cite: 132, 178].
* [cite_start]**`a`**: 6-position Accelerometer calibration[cite: 137, 179].
* [cite_start]**`m`**: Magnetometer calibration (20-second rotation window)[cite: 151, 152, 180].
* [cite_start]**`p`**: Print current calibration values to Serial[cite: 125, 182].

### 3. ESC Calibration Handshake
[cite_start]The flight controller uses a state machine to define throttle limits (920µs to 2120µs)[cite: 87, 189]:
1. [cite_start]**Low Trigger:** Hold throttle below 15% (`THR_SIG_LOW`) to initiate[cite: 90, 190].
2. [cite_start]**High Trigger:** Push throttle above 85% (`THR_SIG_HIGH`) within 10 seconds[cite: 90, 93, 193].
3. [cite_start]**Pulse:** The controller sends a 4-second calibration pulse (`ESC_MAX_US`) to all motors[cite: 91, 195].
4. [cite_start]**Completion:** Once `ESC_CAL_DONE` is reached, normal flight is enabled[cite: 198].

---

## 🎮 Flight Control Logic

[cite_start]The system uses a **Double PID** loop to translate stick angles into motor thrust[cite: 172, 173].



* [cite_start]**Angle PID (Outer):** Converts desired angle into a target rotation rate[cite: 219, 220].
* [cite_start]**Rate PID (Inner):** Converts target rate into motor PWM offsets, capped at ±300µs[cite: 173, 222].
* [cite_start]**Refresh Rate:** The remote sends RC packets at ~40Hz (every 25ms)[cite: 11, 71].
* [cite_start]**Failsafe:** Motors are immediately stopped if the RC signal is lost for more than 200ms[cite: 88, 209].

---

## ⚠️ Safety Information
* [cite_start]**Propellers:** Always remove propellers during ESC and sensor calibration[cite: 183].
* [cite_start]**Arming:** The system requires the arm flag to be active and follows safety checks before spinning motors[cite: 10, 50, 216].
* **Spectators:** Last year's competition had 500 spectators; always maintain a safe distance during operation.