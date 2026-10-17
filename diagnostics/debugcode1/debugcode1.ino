/*
  APSC 200 belt-indexed station controller
  Debug version with explicit motor/encoder sign settings.

  Why this version exists:
  If the motor keeps spinning and the reported angle does not move toward the
  target, the closed loop is not seeing the correct sign relationship between:
    1) commanded motor direction
    2) encoder counting direction
    3) desired positive output angle convention

  This version keeps the same pins, but adds clear sign constants and prints
  raw encoder position too.
*/

#include "RP2040_PWM.h"
#include "PicoEncoder.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "ArduPID.h"
#pragma GCC diagnostic pop

#define encAPin 16
#define encBPin 17
#define AIN1Pin 12
#define AIN2Pin 13
#define STBYPin 14
#define pwmPin  15

const int MIN_EFFECTIVE_DUTY    = 15;
const int MAX_DUTY              = 100;
const uint32_t SERIAL_OUTPUT_MS = 250;
static const float PWM_FREQ_HZ  = 1500.0f;
static const uint16_t BRAKE_MS  = 250;

const float  COUNTS_PER_REV         = 374.22f;
const double STEPS_PER_REV          = COUNTS_PER_REV * 4.0;
const double SUBSTEPS_PER_REV       = STEPS_PER_REV * 64.0;
static const double DEG_PER_SUBSTEP = 360.0 / SUBSTEPS_PER_REV;

static const double DRIVER_DIAMETER_CM = 4.0;
static const double DRIVEN_DIAMETER_CM = 7.5;
static const double MOTOR_DEG_PER_OUTPUT_DEG = DRIVEN_DIAMETER_CM / DRIVER_DIAMETER_CM;

// ------------------ IMPORTANT DEBUG SIGN SWITCHES ------------------
// Leave these defaults first. If the motor runs away, try flipping MOTOR_SIGN.
// If raw encoder counts move opposite to expected, flip ENCODER_SIGN.
static const double MOTOR_SIGN   = +1.0; // +1 keeps original AIN1/AIN2 mapping, -1 swaps it
static const double ENCODER_SIGN = +1.0; // +1 keeps encoder sign, -1 flips measured angle sign

// Open belt, same rotation direction between small and large caps.
static const double OUTPUT_SIGN_RELATIVE_TO_MOTOR = +1.0;

static const double OUTPUT_PICKING_DEG =   0.0;
static const double OUTPUT_LASER_DEG   = 108.0;
static const double OUTPUT_MAPPING_DEG = -90.0;

static const uint32_t LASER_DWELL_MS = 13500UL;
static const uint32_t MAP_DWELL_MS   =  9000UL;

static const double OUTPUT_TOL_DEG = 1.0;
static const double MOTOR_TOL_DEG  = OUTPUT_TOL_DEG * MOTOR_DEG_PER_OUTPUT_DEG;

static double setpointDeg = 0.0;
static double inputDeg    = 0.0;
static double outputDuty  = 0.0;

static double Kp = 0.40;
static double Ki = 0.002;
static double Kd = 0.05;

RP2040_PWM* motorPWM = nullptr;
PicoEncoder  encoder;
ArduPID      myPid;

enum SequenceState {
  MOVE_TO_LASER,
  DWELL_AT_LASER,
  MOVE_TO_MAPPING,
  DWELL_AT_MAPPING,
  MOVE_TO_PICKING,
  CYCLE_COMPLETE,
  ABORTED
};

static SequenceState state;
static uint32_t stateStartMs = 0;
static uint32_t lastPrintMs  = 0;

void motorStop();
void motorBrake();
void applySignedDuty(double dutySigned);
void startCycle();
void beginMoveToOutputAngle(double outputAngleDegCW);
void enterDwell(SequenceState dwellState);
void handleMoveState(SequenceState completedMoveNextState);
void handleDwellState(uint32_t dwellMs, double nextOutputAngleDegCW, SequenceState nextMoveState);
void finishCycle();
void abortCycle();
void printBanner();
void printStateHeader(const char* label, double outputAngleDegCW);
void printMoveUpdate(const char* label, double motorErrDeg);
String readLineNonBlocking();

double outputToMotorDeg(double outputAngleDegCW) {
  return outputAngleDegCW * MOTOR_DEG_PER_OUTPUT_DEG * OUTPUT_SIGN_RELATIVE_TO_MOTOR;
}

double motorToOutputDeg(double motorAngleDeg) {
  return motorAngleDeg / (MOTOR_DEG_PER_OUTPUT_DEG * OUTPUT_SIGN_RELATIVE_TO_MOTOR);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(AIN1Pin, OUTPUT);
  pinMode(AIN2Pin, OUTPUT);
  pinMode(STBYPin, OUTPUT);
  digitalWrite(AIN1Pin, LOW);
  digitalWrite(AIN2Pin, LOW);
  digitalWrite(STBYPin, LOW);

  motorPWM = new RP2040_PWM(pwmPin, PWM_FREQ_HZ, 0.0f);
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);

  if (encoder.begin(encAPin) != 0) {
    Serial.println("ERROR: Encoder init failed.");
    while (true) { delay(1000); }
  }

  encoder.resetPosition();

  myPid.begin(&inputDeg, &outputDuty, &setpointDeg, Kp, Ki, Kd);
  myPid.setOutputLimits(-MAX_DUTY, MAX_DUTY);
  myPid.setWindUpLimits(-10, 10);
  myPid.stop();

  printBanner();
  startCycle();
}

void loop() {
  encoder.update();
  inputDeg = ENCODER_SIGN * ((double)encoder.position) * DEG_PER_SUBSTEP;

  String line = readLineNonBlocking();
  if (line.length() > 0) {
    line.trim();
    if (line.equalsIgnoreCase("x")) { abortCycle(); return; }
    if (line.equalsIgnoreCase("r")) { startCycle(); return; }
  }

  switch (state) {
    case MOVE_TO_LASER:   handleMoveState(DWELL_AT_LASER); break;
    case DWELL_AT_LASER:  handleDwellState(LASER_DWELL_MS, OUTPUT_MAPPING_DEG, MOVE_TO_MAPPING); break;
    case MOVE_TO_MAPPING: handleMoveState(DWELL_AT_MAPPING); break;
    case DWELL_AT_MAPPING:handleDwellState(MAP_DWELL_MS, OUTPUT_PICKING_DEG, MOVE_TO_PICKING); break;
    case MOVE_TO_PICKING: handleMoveState(CYCLE_COMPLETE); break;
    case CYCLE_COMPLETE:
    case ABORTED:
    default: break;
  }
}

void startCycle() {
  motorStop();
  encoder.update();
  encoder.resetPosition();
  inputDeg = 0.0;
  setpointDeg = 0.0;
  outputDuty = 0.0;
  myPid.stop();
  myPid.reset();
  stateStartMs = millis();
  lastPrintMs = 0;
  Serial.println();
  Serial.println("Cycle starting from PICKING station (0 deg).");
  Serial.println("Type 'x' + Enter to abort. Type 'r' + Enter to restart after re-homing.");
  beginMoveToOutputAngle(OUTPUT_LASER_DEG);
  state = MOVE_TO_LASER;
}

void beginMoveToOutputAngle(double outputAngleDegCW) {
  setpointDeg = outputToMotorDeg(outputAngleDegCW);
  myPid.reset();
  myPid.start();
  stateStartMs = millis();
  lastPrintMs = 0;
  if (outputAngleDegCW == OUTPUT_LASER_DEG) printStateHeader("Moving to LASER station", outputAngleDegCW);
  else if (outputAngleDegCW == OUTPUT_MAPPING_DEG) printStateHeader("Moving to SURFACE MAPPING station", outputAngleDegCW);
  else printStateHeader("Returning to PICKING station", outputAngleDegCW);
}

void enterDwell(SequenceState dwellState) {
  motorBrake();
  delay(BRAKE_MS);
  encoder.update();
  inputDeg = ENCODER_SIGN * ((double)encoder.position) * DEG_PER_SUBSTEP;
  motorStop();
  myPid.stop();
  myPid.reset();
  state = dwellState;
  stateStartMs = millis();
  lastPrintMs = 0;
  if (dwellState == DWELL_AT_LASER) Serial.println("Reached LASER station. Dwelling for 13.5 s.");
  else Serial.println("Reached SURFACE MAPPING station. Dwelling for 9.0 s.");
}

void handleMoveState(SequenceState completedMoveNextState) {
  const double motorErrDeg = setpointDeg - inputDeg;
  if (fabs(motorErrDeg) <= MOTOR_TOL_DEG) {
    if (completedMoveNextState == DWELL_AT_LASER || completedMoveNextState == DWELL_AT_MAPPING) enterDwell(completedMoveNextState);
    else finishCycle();
    return;
  }

  myPid.compute();
  double dutyCmd = outputDuty;
  if (fabs(dutyCmd) > 0.0 && fabs(dutyCmd) < MIN_EFFECTIVE_DUTY) {
    dutyCmd = (dutyCmd > 0.0) ? MIN_EFFECTIVE_DUTY : -MIN_EFFECTIVE_DUTY;
  }
  applySignedDuty(dutyCmd);

  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= SERIAL_OUTPUT_MS) {
    lastPrintMs = nowMs;
    if (state == MOVE_TO_LASER) printMoveUpdate("LASER", motorErrDeg);
    else if (state == MOVE_TO_MAPPING) printMoveUpdate("MAPPING", motorErrDeg);
    else printMoveUpdate("PICKING", motorErrDeg);
  }
}

void handleDwellState(uint32_t dwellMs, double nextOutputAngleDegCW, SequenceState nextMoveState) {
  uint32_t elapsedMs = millis() - stateStartMs;
  uint32_t remainingMs = (elapsedMs >= dwellMs) ? 0UL : (dwellMs - elapsedMs);
  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= 1000UL) {
    lastPrintMs = nowMs;
    Serial.print("Dwelling... ");
    Serial.print(remainingMs / 1000.0, 1);
    Serial.println(" s remaining");
  }
  if (elapsedMs >= dwellMs) {
    state = nextMoveState;
    beginMoveToOutputAngle(nextOutputAngleDegCW);
  }
}

void finishCycle() {
  motorBrake(); delay(BRAKE_MS); encoder.update();
  inputDeg = ENCODER_SIGN * ((double)encoder.position) * DEG_PER_SUBSTEP;
  motorStop(); myPid.stop(); myPid.reset();
  state = CYCLE_COMPLETE;
  Serial.println();
  Serial.println("Cycle complete.");
  Serial.print("Final output angle: ");
  Serial.print(motorToOutputDeg(inputDeg), 3);
  Serial.println(" deg");
  Serial.println("Place lever at the picking station, then type 'r' + Enter to run again.");
}

void abortCycle() {
  motorBrake(); delay(BRAKE_MS); encoder.update();
  inputDeg = ENCODER_SIGN * ((double)encoder.position) * DEG_PER_SUBSTEP;
  motorStop(); myPid.stop(); myPid.reset();
  state = ABORTED;
  Serial.println();
  Serial.println("Cycle aborted.");
  Serial.print("Current output angle: ");
  Serial.print(motorToOutputDeg(inputDeg), 3);
  Serial.println(" deg");
  Serial.println("Manually re-home to the picking station, then type 'r' + Enter to restart.");
}

void printBanner() {
  Serial.println();
  Serial.println("APSC 200 belt-indexed station controller (debug version)");
  Serial.print("Driver cap diameter  = "); Serial.print(DRIVER_DIAMETER_CM, 2); Serial.println(" cm");
  Serial.print("Driven cap diameter  = "); Serial.print(DRIVEN_DIAMETER_CM, 2); Serial.println(" cm");
  Serial.print("Motor deg / output deg = "); Serial.println(MOTOR_DEG_PER_OUTPUT_DEG, 4);
  Serial.print("MOTOR_SIGN = "); Serial.println(MOTOR_SIGN, 0);
  Serial.print("ENCODER_SIGN = "); Serial.println(ENCODER_SIGN, 0);
  Serial.println("Assumption: open belt => output rotates same direction as motor.");
}

void printStateHeader(const char* label, double outputAngleDegCW) {
  Serial.println();
  Serial.println(label);
  Serial.print("Target output angle (large cap / lever): "); Serial.print(outputAngleDegCW, 2); Serial.println(" deg");
  Serial.print("Target motor angle: "); Serial.print(outputToMotorDeg(outputAngleDegCW), 2); Serial.println(" deg");
}

void printMoveUpdate(const char* label, double motorErrDeg) {
  const double currentOutputDeg = motorToOutputDeg(inputDeg);
  const double targetOutputDeg  = motorToOutputDeg(setpointDeg);
  const double errorOutputDeg   = targetOutputDeg - currentOutputDeg;
  Serial.print("Moving to "); Serial.print(label);
  Serial.print(" | raw enc: "); Serial.print((long)encoder.position);
  Serial.print(" | Motor pos: "); Serial.print(inputDeg, 2);
  Serial.print(" deg | Motor err: "); Serial.print(motorErrDeg, 2);
  Serial.print(" deg | Output pos: "); Serial.print(currentOutputDeg, 2);
  Serial.print(" deg | Output err: "); Serial.print(errorOutputDeg, 2);
  Serial.print(" deg | Duty cmd: "); Serial.print(outputDuty, 2);
  Serial.println();
}

void motorStop() {
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);
  digitalWrite(AIN1Pin, LOW);
  digitalWrite(AIN2Pin, LOW);
  digitalWrite(STBYPin, LOW);
}

void motorBrake() {
  digitalWrite(STBYPin, HIGH);
  digitalWrite(AIN1Pin, HIGH);
  digitalWrite(AIN2Pin, HIGH);
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);
}

void applySignedDuty(double dutySigned) {
  if (dutySigned >  100.0) dutySigned =  100.0;
  if (dutySigned < -100.0) dutySigned = -100.0;
  uint8_t duty = (uint8_t)(fabs(dutySigned) + 0.5);
  if (duty == 0) {
    digitalWrite(STBYPin, HIGH);
    motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);
    return;
  }
  digitalWrite(STBYPin, HIGH);
  if ((dutySigned * MOTOR_SIGN) >= 0.0) {
    digitalWrite(AIN1Pin, HIGH);
    digitalWrite(AIN2Pin, LOW);
  } else {
    digitalWrite(AIN1Pin, LOW);
    digitalWrite(AIN2Pin, HIGH);
  }
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, (float)duty);
}

String readLineNonBlocking() {
  static String buf = "";
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = buf;
      buf = "";
      line.trim();
      return line;
    }
    buf += c;
  }
  return "";
}
