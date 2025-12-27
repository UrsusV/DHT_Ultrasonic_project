================================================================================
DHT11 & Ultrasonic Sensor Project with Servo Motor Control
CSE331L - Microprocessors and Embedded Systems
================================================================================

PROJECT OVERVIEW
================================================================================
This embedded systems project implements a dual-mode environmental monitoring
and control system using the STM32F103C8TX microcontroller. The system monitors
temperature/humidity and distance measurements, displaying readings on an LCD
and controlling a servo motor based on configurable thresholds.

================================================================================
HARDWARE COMPONENTS
================================================================================

Microcontroller:
- STM32F103C8TX (ARM Cortex-M3, 72MHz)

Sensors:
- DHT11 Temperature and Humidity Sensor
  * Connection: PA1 (Data pin)
  * Range: Temperature 0-50°C, Humidity 20-90% RH
  * Update Rate: Every 2 seconds

- HC-SR04 Ultrasonic Distance Sensor
  * Trigger: PA9 (GPIO Output)
  * Echo: PA10 (GPIO Input)
  * Range: 2-400 cm
  * Update Rate: Every 500ms

Actuators:
- SG90 Servo Motor
  * Connection: PA0 (PWM via TIM2 Channel 1)
  * Function: Visual indicator (sweep motion when active)

Display:
- I2C LCD Display (16x2)
  * Connection: I2C1 (PB6-SCL, PB7-SDA)
  * Displays sensor readings and system status

Input:
- 2-Position Switch
  * Connection: PB0 (with internal pull-up)
  * Function: Mode selection between Temperature and Distance modes

- 3x4 Matrix Keypad
  * Connection: PA2-PA8
  * Note: Connected but not actively used in current implementation

================================================================================
SYSTEM FUNCTIONALITY
================================================================================

Operating Modes:
1. Temperature Mode (Switch Released/HIGH - PB0 = HIGH)
   - Displays DHT11 temperature and humidity readings
   - Updates every 2 seconds
   - Activates servo motor when temperature ≥ 30°C

2. Distance Mode (Switch Pressed/LOW - PB0 = LOW)
   - Displays HC-SR04 ultrasonic distance measurements
   - Updates every 500ms
   - Activates servo motor when distance ≤ 20cm

Motor Control Logic:
- Servo motor performs sweep motion as a visual indicator
- Temperature Mode: Motor ON when temperature ≥ 30°C
- Distance Mode: Motor ON when distance ≤ 20cm
- Motor returns to center position when inactive

Display Information:
- Temperature Mode:
  * Line 1: Temperature with motor status and threshold indicator
  * Line 2: Humidity percentage and overall motor status

- Distance Mode:
  * Line 1: Distance with motor status and threshold indicator
  * Line 2: Overall motor status

================================================================================
SOFTWARE ARCHITECTURE
================================================================================

Core Modules:
- main.c: Main application logic, mode switching, and system coordination
- dht11.c/h: DHT11 sensor driver with temperature and humidity reading functions
- sonic.c/h: HC-SR04 ultrasonic sensor driver with distance measurement
- sg90.c/h: SG90 servo motor control with PWM-based positioning
- i2c-lcd.c/h: I2C LCD display driver for 16x2 character display
- switch.c/h: Switch interface for mode selection with debouncing

Key Features:
- Real-time sensor data acquisition
- Dual-mode operation with smooth mode transitions
- Threshold-based motor control
- LCD display with formatted sensor data
- Switch debouncing (300ms) for reliable mode switching
- Error handling for sensor communication failures

================================================================================
CONFIGURATION
================================================================================

Thresholds (defined in main.c):
- TEMP_THRESHOLD: 30.0°C (temperature trigger for motor activation)
- DISTANCE_THRESHOLD: 20.0 cm (distance trigger for motor activation)

Peripheral Configuration:
- System Clock: 72 MHz (HSE with PLL)
- TIM2: PWM generation for servo motor (20ms period, 1.5ms center pulse)
- I2C1: LCD communication (100kHz standard mode)
- GPIO: Sensor and switch interfaces with appropriate pull-up/down settings

================================================================================
HARDWARE CONNECTIONS SUMMARY
================================================================================

Port A (GPIOA):
- PA0: Servo Motor PWM (TIM2_CH1)
- PA1: DHT11 Data
- PA2-PA8: Keypad (3x4 matrix, not actively used)
- PA9: Ultrasonic Trigger
- PA10: Ultrasonic Echo

Port B (GPIOB):
- PB0: Mode Selection Switch (with internal pull-up)
- PB6: I2C1 SCL (LCD)
- PB7: I2C1 SDA (LCD)

Power:
- VCC: 5V for sensors and servo motor
- GND: Common ground
- STM32: 3.3V logic levels (with appropriate level conversion if needed)

================================================================================
DEVELOPMENT ENVIRONMENT
================================================================================

- IDE: STM32CubeIDE (or compatible STM32 development environment)
- HAL Library: STM32F1xx HAL Driver
- Compiler: ARM GCC
- Debugger: ST-Link or compatible SWD interface

================================================================================
PROJECT STRUCTURE
================================================================================

Core/
  ├── Inc/          Header files for all modules
  ├── Src/          Source files for application code
  └── Startup/      STM32 startup code and vector table

Drivers/
  ├── CMSIS/        Cortex Microcontroller Software Interface Standard
  └── STM32F1xx_HAL_Driver/  STM32 HAL library files

Debug/              Build output and object files

================================================================================
NOTES
================================================================================

- The system implements mode-based operation where only the active sensor's
  reading is used for motor control, ensuring accurate threshold-based
  responses for each mode.

- Sensor readings include error detection and display appropriate error
  messages on the LCD when communication fails.

- The servo motor sweep motion provides a clear visual indication of threshold
  conditions being met.

- Switch debouncing ensures reliable mode switching without false triggers.

================================================================================

