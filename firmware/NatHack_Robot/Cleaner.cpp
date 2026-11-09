#include "Cleaner.h"
#include "Config.h"
#include "MotorControl.h"
#include "Actuators.h"
#include "Protocol.h"

enum class Step : uint8_t {
  IDLE, APPROACH, SPRAY_LIFT, SPRAYING, SPRAY_LOWER,
  GROUND_FORWARD, GROUND_BACKWARD, WALL_WIPE, FINISHED
};
static Step step = Step::IDLE;
static Surface targetSurface = Surface::NONE;
static Fluid targetFluid = Fluid::NONE;
static unsigned long stepAt = 0;
static int cycle = 0;
static int spongeAngle = SPONGE_HOME_DEG;
static int spongeDirection = 1;
static int completedSweeps = 0;

static void enterStep(Step next) { step = next; stepAt = millis(); }
static void finish() {
  motorsStop(); pumpsOff();
  sprayServoAngle(SPRAY_HOME_DEG);
  spongeServoAngle(SPONGE_HOME_DEG);
  targetSurface = Surface::NONE; targetFluid = Fluid::NONE;
  enterStep(Step::FINISHED);
  sendReply("DONE,CLEANING");
  enterStep(Step::IDLE);
}

void cleanerBegin() { cleanerAbort(); }
void cleanerAbort() {
  motorsStop(); pumpsOff();
  sprayServoAngle(SPRAY_HOME_DEG);
  spongeServoAngle(SPONGE_HOME_DEG);
  targetSurface = Surface::NONE;
  targetFluid = Fluid::NONE;
  enterStep(Step::IDLE);
}

bool cleanerSetTarget(Surface surface, Fluid fluid) {
  if (step != Step::IDLE || surface == Surface::NONE || fluid == Fluid::NONE) return false;
  targetSurface = surface;
  targetFluid = fluid;
  enterStep(Step::APPROACH);
  return true;
}

bool cleanerStart() {
  if (step != Step::APPROACH) return false;
  motorsStop(); cycle = 0;
  if (targetSurface == Surface::WALL) {
    sprayServoAngle(SPRAY_WALL_DEG);
    enterStep(Step::SPRAY_LIFT);
  } else {
    pumpSelect(targetFluid == Fluid::BLOOD ? 'B' : 'O');
    enterStep(Step::SPRAYING);
  }
  sendReply("EVENT,CLEANING_STARTED");
  return true;
}

void cleanerUpdate() {
  unsigned long now = millis();
  switch (step) {
    case Step::SPRAY_LIFT:
      if (now - stepAt >= SPRAY_SETTLE_MS) {
        pumpSelect(targetFluid == Fluid::BLOOD ? 'B' : 'O');
        enterStep(Step::SPRAYING);
      }
      break;
    case Step::SPRAYING:
      if (now - stepAt >= SPRAY_MS) {
        pumpsOff();
        if (targetSurface == Surface::WALL) {
          sprayServoAngle(SPRAY_HOME_DEG);
          enterStep(Step::SPRAY_LOWER);
        } else {
          motorsDrive('F', CLEAN_DRIVE_PWM, false);
          enterStep(Step::GROUND_FORWARD);
        }
      }
      break;
    case Step::SPRAY_LOWER:
      if (now - stepAt >= SPRAY_SETTLE_MS) {
        spongeAngle = SPONGE_HOME_DEG;
        spongeDirection = 1;
        completedSweeps = 0;
        enterStep(Step::WALL_WIPE);
      }
      break;
    case Step::GROUND_FORWARD:
      if (now - stepAt >= GROUND_STROKE_MS) {
        motorsDrive('B', CLEAN_DRIVE_PWM, false);
        enterStep(Step::GROUND_BACKWARD);
      }
      break;
    case Step::GROUND_BACKWARD:
      if (now - stepAt >= GROUND_STROKE_MS) {
        cycle++;
        if (cycle >= GROUND_STROKE_CYCLES) finish();
        else {
          motorsDrive('F', CLEAN_DRIVE_PWM, false);
          enterStep(Step::GROUND_FORWARD);
        }
      }
      break;
    case Step::WALL_WIPE:
      if (now - stepAt >= WALL_SERVO_STEP_MS) {
        stepAt = now;
        spongeAngle += spongeDirection * WALL_SERVO_STEP_DEG;
        if (spongeAngle >= SPONGE_MAX_DEG) {
          spongeAngle = SPONGE_MAX_DEG;
          spongeDirection = -1;
        } else if (spongeAngle <= SPONGE_HOME_DEG) {
          spongeAngle = SPONGE_HOME_DEG;
          spongeDirection = 1;
          completedSweeps++;
        }
        spongeServoAngle(spongeAngle);
        if (completedSweeps >= WALL_WIPE_CYCLES) finish();
      }
      break;
    default: break;
  }
}

bool cleanerBusy() { return step != Step::IDLE && step != Step::APPROACH; }
bool cleanerHasTarget() { return step != Step::IDLE; }
const char* cleanerStateName() {
  switch (step) {
    case Step::IDLE: return "IDLE";
    case Step::APPROACH: return "APPROACH";
    case Step::SPRAY_LIFT: return "SPRAY_LIFT";
    case Step::SPRAYING: return "SPRAYING";
    case Step::SPRAY_LOWER: return "SPRAY_LOWER";
    case Step::GROUND_FORWARD: return "GROUND_FORWARD";
    case Step::GROUND_BACKWARD: return "GROUND_BACKWARD";
    case Step::WALL_WIPE: return "WALL_WIPE";
    case Step::FINISHED: return "FINISHED";
  }
  return "UNKNOWN";
}
