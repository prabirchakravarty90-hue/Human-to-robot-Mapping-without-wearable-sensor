TowerPro MG996R Digital High Torque Servo Motor – 180° (x5)
High-torque, metal-gear servo motors for precise angular motion.
PCA9685 – 16-Channel 12-bit PWM/Servo Driver (I2C interface)
Provides stable PWM signals to control multiple servos simultaneously.
ESP32 WiFi + Bluetooth Development Board
Main controller with dual-core processing, Wi-Fi, and Bluetooth connectivity.
Jumper Wires & Single-Stranded Wires
For making reliable electrical connections between components.

⚡ System Overview
ESP32 communicates with the PCA9685 over I2C.
PCA9685 generates 50 Hz PWM signals to control up to 16 servo motors.
MG996R Servos are powered through an external 5V power supply (not directly from ESP32).
common Ground is shared between ESP32, PCA9685, and the servo power supply.

🔌 Basic Wiring
ESP32 → PCA9685
SDA → SDA
SCL → SCL
VCC → 3.3V/5V
GND → GND
PCA9685 → Servo Motors
PWM pins → Servo signal (orange/yellow)
V+ → Servo power (red, 5–6V external supply)
GND → Servo ground (black/brown)
