#pragma once
#include <Arduino.h>

// CHANGE THESE BEFORE UPLOADING.
constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
constexpr uint16_t NET_PORT = 3333;

// EXAMPLE PIN PLAN ONLY. VERIFY YOUR ACTUAL ESP32-S3 BOARD + EXISTING WIRING.
// Motor indexes: FL=front-left, RL=rear-left, FR=front-right, RR=rear-right.
// Driver 1 channel A: FL; channel B: RL.
// Driver 2 channel A: FR; channel B: RR.
struct MotorPins { int in1, in2, pwm; bool invert; };
constexpr MotorPins MOTOR_PINS[4] = {
  {1, 2, 11, false}, // Front left
  {3, 4, 12, false}, // Rear left
  {5, 6, 13, false}, // Front right
  {7, 8, 14, false}  // Rear right
};
constexpr int STBY_DRIVER_1 = 15;
constexpr int STBY_DRIVER_2 = 16;

// Pump GPIOs drive external MOSFET/relay INPUTS, NEVER pump power directly!
constexpr int PUMP_A_PIN = 17; // Blood solution
constexpr int PUMP_B_PIN = 18; // Other solution
constexpr bool PUMP_ACTIVE_HIGH = true;

// Servo signal pins, matching your last test.
constexpr int SPRAY_SERVO_PIN = 9;  // Lift BOTH spray tubes together
constexpr int SPONGE_SERVO_PIN = 10; // Separate wall sponge
constexpr int SPRAY_HOME_DEG = 0;
constexpr int SPRAY_WALL_DEG = 90;  // Calibrate actual angle
constexpr int SPONGE_HOME_DEG = 0;
constexpr int SPONGE_MAX_DEG = 180; // Use a smaller angle if linkage binds

// Tune on the real robot, starting with safe low speeds.
constexpr int MAX_PWM = 255;
constexpr int CLEAN_DRIVE_PWM = 110;
constexpr unsigned long MOVE_WATCHDOG_MS = 450;
constexpr unsigned long SPRAY_MS = 1200;
constexpr unsigned long SPRAY_SETTLE_MS = 700;
constexpr unsigned long GROUND_STROKE_MS = 350;
constexpr int GROUND_STROKE_CYCLES = 3;
constexpr unsigned long WALL_SERVO_STEP_MS = 15;
constexpr int WALL_SERVO_STEP_DEG = 2;
constexpr int WALL_WIPE_CYCLES = 3;
