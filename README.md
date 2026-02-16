# ESP32 Drone & Remote Control System

This project is a high-performance quadcopter system utilizing **ESP-NOW** for low-latency communication. It features a dual-PID control architecture (Angle + Rate) and a custom remote controller with OLED telemetry and logarithmic stick mapping.

---

## 🛠 Hardware Configuration

### Remote Controller Wiring
The remote uses a 12-bit ADC resolution (0..4095) for precise stick input.

| Component | ESP32 Pin | Notes |
| :--- | :--- | :--- |
| **OLED SDA** | GPIO 21 | I2C Data |
| **OLED SCL** | GPIO 22 | I2C Clock (400kHz) |
| **Joystick X** | GPIO 35 | Roll axis |
| **Joystick Y** | GPIO 34 | Pitch axis |
| **Joystick Button** | GPIO 32 | Digital Input |
| **Throttle Pot** | GPIO 39 | Optional analog throttle |
| **Yaw Pot** | GPIO 36 | Optional analog yaw |

### Drone (Flight Controller) Wiring
The motors follow a **Quad-X** configuration with specific PWM limits.



| Motor | ESP32 Pin | Rotation |
| :--- | :--- | :--- |
| **Front-Left (FL)** | GPIO 33 | CW |
| **Front-Right (FR)** | GPIO 25 | CCW |
| **Back-Left (BL)** | GPIO 32 | CCW |
| **Back-Right (BR)** | GPIO 4 | CW |

---

## 🚀 Setup & Calibration

### 1. Remote Preparation
* **MAC Address:** Upon boot, the remote prints its MAC address to the Serial Monitor. Ensure this matches the `remoteMac` defined in the flight controller code.
* **Joystick Calibration:** On the first run, the system calibrates centers and saves them to preferences. To force recalibration, hold the joystick button while powering on.
* **Curve Tuning:** Stick sensitivity is refined via logarithmic mapping (adjustable via `LOG_K` constants).

### 2. Drone Sensor Calibration
Access the Serial menu (115200 baud) and use the following commands:
* **`g`**: Calibrate Gyro (Keep the board perfectly still).
* **`a`**: 6-position Accelerometer calibration.
* **`m`**: Magnetometer calibration (20-second rotation window).
* **`p`**: Print current calibration values to Serial.

### 3. ESC Calibration Handshake
The flight controller uses a state machine to define throttle limits (920µs to 2120µs):
1. **Low Trigger:** Hold throttle below 15% to initiate the sequence.
2. **High Trigger:** Push throttle above 85% within 10 seconds.
3. **Pulse:** The controller sends a 4-second calibration pulse (2120µs) to all motors.
4. **Completion:** Once the cycle finishes, normal flight is enabled.

---

## 🎮 Flight Control Logic

The system uses a **Double PID** loop to translate stick angles into motor thrust.



* **Angle PID (Outer):** Converts desired angle into a target rotation rate.
* **Rate PID (Inner):** Converts target rate into motor PWM offsets, capped at ±300µs.
* **Refresh Rate:** The remote sends RC packets at ~40Hz (every 25ms).
* **Failsafe:** Motors are immediately stopped if the RC signal is lost for more than 200ms.

---

## ⚠️ Safety Information
* **Propellers:** Always remove propellers during ESC and sensor calibration.
* **Arming:** The system requires an arming signal and checks for low throttle before spinning motors.
* **Environment:** Ensure you test in a controlled, clear area.