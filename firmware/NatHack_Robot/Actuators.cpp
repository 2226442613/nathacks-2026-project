#include "Actuators.h"
#include "Config.h"
#include <ESP32Servo.h>

static Servo sprayServo;
static Servo spongeServo;

static void writePump(int pin, bool on) {
  digitalWrite(pin, (on == PUMP_ACTIVE_HIGH) ? HIGH : LOW);
}

void pumpsOff() {
  writePump(PUMP_A_PIN, false);
  writePump(PUMP_B_PIN, false);
}

void actuatorsBegin() {
  pinMode(PUMP_A_PIN, OUTPUT);
  pinMode(PUMP_B_PIN, OUTPUT);
  pumpsOff();
  sprayServo.setPeriodHertz(50);
  spongeServo.setPeriodHertz(50);
  sprayServo.attach(SPRAY_SERVO_PIN, 500, 2400);
  spongeServo.attach(SPONGE_SERVO_PIN, 500, 2400);
  sprayServoAngle(SPRAY_HOME_DEG);
  spongeServoAngle(SPONGE_HOME_DEG);
}

void pumpSelect(char fluid) {
  pumpsOff(); // Never enable two fluids together.
  if (fluid == 'B') writePump(PUMP_A_PIN, true);
  if (fluid == 'O') writePump(PUMP_B_PIN, true);
}

void sprayServoAngle(int degrees) {
  sprayServo.write(constrain(degrees, 0, 180));
}

void spongeServoAngle(int degrees) {
  spongeServo.write(constrain(degrees, 0, 180));
}
