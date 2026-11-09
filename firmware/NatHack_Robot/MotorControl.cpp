#include "MotorControl.h"
#include "Config.h"

static unsigned long lastMoveAt = 0;
static bool timedMotion = false;

static void setMotor(int index, int signedPower) {
  const MotorPins &m = MOTOR_PINS[index];
  if (m.invert) signedPower = -signedPower;
  signedPower = constrain(signedPower, -MAX_PWM, MAX_PWM);
  digitalWrite(m.in1, signedPower > 0 ? HIGH : LOW);
  digitalWrite(m.in2, signedPower < 0 ? HIGH : LOW);
  analogWrite(m.pwm, abs(signedPower));
}

void motorsBegin() {
  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PINS[i].in1, OUTPUT);
    pinMode(MOTOR_PINS[i].in2, OUTPUT);
    pinMode(MOTOR_PINS[i].pwm, OUTPUT);
    setMotor(i, 0);
  }
  pinMode(STBY_DRIVER_1, OUTPUT);
  pinMode(STBY_DRIVER_2, OUTPUT);
  digitalWrite(STBY_DRIVER_1, HIGH);
  digitalWrite(STBY_DRIVER_2, HIGH);
}

void motorsStop() {
  for (int i = 0; i < 4; i++) setMotor(i, 0);
  timedMotion = false;
}

void motorsDrive(char direction, int pwm, bool timed) {
  pwm = constrain(pwm, 0, MAX_PWM);
  int left = 0, right = 0;
  switch (direction) {
    case 'F': left = pwm; right = pwm; break;
    case 'B': left = -pwm; right = -pwm; break;
    case 'L': left = -pwm; right = pwm; break;
    case 'R': left = pwm; right = -pwm; break;
    default: motorsStop(); return;
  }
  setMotor(0, left); setMotor(1, left);
  setMotor(2, right); setMotor(3, right);
  timedMotion = timed;
  lastMoveAt = millis();
}

void motorsUpdate() {
  if (timedMotion && millis() - lastMoveAt > MOVE_WATCHDOG_MS) motorsStop();
}
