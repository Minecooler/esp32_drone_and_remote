# esp32_drone_and_remote
🚁 Drone & Remote System Overview
This project consists of a quadcopter flight controller and a dedicated remote controller, both powered by ESP32 microcontrollers communicating via the ESP-NOW protocol. The system features a double-PID control loop (Angle + Rate) for stable flight and an integrated OLED feedback system on the remote.

🔌 Hardware Connections
Drone (Flight Controller)
MCU: ESP32


Sensors: L3G Gyro, LSM303 Accel/Mag (connected via I2C) 
+1

Motors (Quad-X):


FL (Front-Left): GPIO 33 
+1


FR (Front-Right): GPIO 25 


BL (Back-Left): GPIO 32 


BR (Back-Right): GPIO 4 

Remote Controller
MCU: ESP32


Display: SSD1306/SH1106 OLED (128x64) 
+2


Inputs: * Joystick X: GPIO 35 


Joystick Y: GPIO 34 


Button: GPIO 32 


Throttle Pot: GPIO 39 


Yaw Pot: GPIO 36 

🛠️ Setup & Installation
1. Flight Controller Setup
Open DoublePID_FlightController.ino in the Arduino IDE.

Ensure you have the following libraries installed:

L3G, LSM303, SensorFusion, Preferences.


Check MAC Address: Ensure the remoteMac in the drone code  matches the MAC address of your remote ESP32 (printed to Serial upon remote boot).

Upload the code to the drone's ESP32.

2. Remote Controller Setup
Open Remote_ESP32_HU_M16_OLED.ino.

Ensure you have U8g2, Adafruit_GFX, and Adafruit_SSD1306 libraries.

The remote uses I2C pins 21 (SDA) and 22 (SCL) for the OLED.
+1

Upload the code to the remote's ESP32.

🚀 Startup & Calibration
Sensor Calibration (Drone)
When connected to USB, use the Serial Monitor (115200 baud) to access the calibration menu:


'g': Calibrate Gyro (keep board perfectly still).
+1


'a': 6-position Accelerometer calibration.
+1


'm': Magnetometer calibration (rotate in all axes).
+1


's': Start the main flight loop.

Joystick Calibration (Remote)
Calibration is performed on the first boot and saved to memory.
+1


Force Recalibration: Hold the joystick button while powering on the remote.

ESC Calibration Handshake
To calibrate your ESCs via the remote:

Power on the drone and remote.


Low Throttle: Pull the throttle stick to the bottom (< 15%).


High Throttle: Push the throttle stick to the top (> 85%) within 10 seconds.

The drone will send a 4-second high pulse followed by a low pulse to set the ESC range (920µs - 2120µs).
+1

⚠️ Safety Warnings
Props Off: Always perform calibrations and initial tests with the propellers removed.


Failsafe: The drone is programmed to stop motors if the RC signal is lost for more than 200ms.
+2


Arming: By default, the drone requires an "Armed" signal from the remote to spin the motors.
+1