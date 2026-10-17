/*
  APSC 200
  Time-based indexed station sequence for a belt-driven cap system.

  This version does NOT use the encoder. It uses calibrated move TIMES to
  estimate the output lever angle.

  Required sequence (output lever):
    1) Start at PICKING station                      ->   0 deg
    2) Rotate 108 deg clockwise to LASER station     -> 108 deg
    3) Dwell 13.5 s at LASER station
    4) Rotate 198 deg counterclockwise to MAPPING    -> -90 deg net angle
    5) Dwell 9.0 s at MAPPING station
    6) Rotate 90 deg clockwise back to PICKING       ->   0 deg

  IMPORTANT LIMITATION
  --------------------
  Because there is no encoder feedback, this sketch can only ESTIMATE angle
  from time. It can meet the dwell-time requirement exactly, but true indexing
  accuracy depends on stable battery voltage, stable load, and correct timing
  calibration on the real mechanism.

  STARTING ASSUMPTION
  -------------------
  Before power-up or before typing 'r' to run again, physically place the
  lever at the PICKING station. This sketch always defines that physical
  location as 0 degrees.

  CALIBRATION
  -----------
  The three move-time constants below are the main tuning knobs:
    LASER_MOVE_MS   : time for 0 deg -> +108 deg (clockwise)
    MAPPING_MOVE_MS : time for +108 deg -> -90 deg (198 deg counterclockwise)
    PICKING_MOVE_MS : time for -90 deg -> 0 deg (90 deg clockwise)

  The default values are only starting guesses based on your earlier tests.
  Tune them on the real installed system if needed.

  SERIAL COMMANDS
  ---------------
    x + Enter  -> abort current cycle immediately
    r + Enter  -> restart after manually re-homing to PICKING station
*/

// Pin assignments kept the same as your encoder-based version
#define AIN1Pin 12
#define AIN2Pin 13
#define STBYPin 14
#define pwmPin  15

// ---------------------------
// Motion calibration constants
// ---------------------------
// These are OPEN-LOOP move times. Adjust them experimentally on your real rig.
static const uint32_t LASER_MOVE_MS   = 360;  // 0 -> +108 deg CW
static const uint32_t MAPPING_MOVE_MS = 520;  // +108 -> -90 deg CCW (198 deg)
static const uint32_t PICKING_MOVE_MS = 300;  // -90 -> 0 deg CW

// Dwell times required by the problem statement
static const uint32_t LASER_DWELL_MS  = 13500UL;
static const uint32_t MAP_DWELL_MS    =  9000UL;

// Output lever absolute station angles (clockwise positive)
static const double OUTPUT_PICKING_DEG =   0.0;
static const double OUTPUT_LASER_DEG   = 108.0;
static const double OUTPUT_MAPPING_DEG = -90.0;

// Duty settings for smooth approach near each stop.
// High duty gets the move going, then duty is reduced near the end.
static const int DUTY_CW_FAST    = 80;
static const int DUTY_CW_MEDIUM  = 55;
static const int DUTY_CW_SLOW    = 32;
static const int DUTY_CCW_FAST   = 82;
static const int DUTY_CCW_MEDIUM = 58;
static const int DUTY_CCW_SLOW   = 35;

// Progress breakpoints for slowing near the target.
// Example: for the first 70% of the move, use FAST duty.
static const double FAST_PHASE_END   = 0.70;
static const double MED_PHASE_END    = 0.92;

// Serial print interval during motion / dwell
static const uint32_t SERIAL_OUTPUT_MS = 250;

// ---------------------------
// State machine
// ---------------------------
enum SequenceState {
  MOVE_TO_LASER,
  DWELL_AT_LASER,
  MOVE_TO_MAPPING,
  DWELL_AT_MAPPING,
  MOVE_TO_PICKING,
  CYCLE_COMPLETE,
  ABORTED
};

static SequenceState state = ABORTED;

// Estimated output angle (because there is no encoder)
static double estimatedOutputDeg = 0.0;

// Current move information
static double moveStartOutputDeg = 0.0;
static double moveTargetOutputDeg = 0.0;
static double moveDeltaOutputDeg = 0.0;
static uint32_t moveDurationMs = 0;
static uint32_t stateStartMs = 0;
static uint32_t lastPrintMs = 0;

// ---------------------------
// Helper declarations
// ---------------------------
void motorStop();
void motorBrake();
void applySignedDuty(int signedDuty);
int  dutyForMoveProgress(double progress, double deltaDeg);

void startCycle();
void startMove(double targetOutputDeg, uint32_t plannedMoveMs, SequenceState moveState);
void enterDwell(SequenceState dwellState);
void handleMoveState(SequenceState dwellStateAfterMove);
void handleDwellState(uint32_t dwellMs, double nextTargetDeg, uint32_t nextMoveMs, SequenceState nextMoveState);
void finishCycle();
void abortCycle();

void printBanner();
void printStateHeader(const char* label, double targetOutputDeg, uint32_t plannedMoveMs);
void printMoveUpdate(const char* label, uint32_t elapsedMs, int dutyCmd);
String readLineNonBlocking();

// ---------------------------
// setup()
// ---------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(AIN1Pin, OUTPUT);
  pinMode(AIN2Pin, OUTPUT);
  pinMode(STBYPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  // Start safe: motor disabled and PWM off
  digitalWrite(AIN1Pin, LOW);
  digitalWrite(AIN2Pin, LOW);
  digitalWrite(STBYPin, LOW);
  analogWrite(pwmPin, 0);

  printBanner();
  startCycle();
}

// ---------------------------
// loop()
// ---------------------------
void loop() {
  String line = readLineNonBlocking();
  if (line.length() > 0) {
    line.trim();

    if (line.equalsIgnoreCase("x")) {
      abortCycle();
      return;
    }

    if (line.equalsIgnoreCase("r")) {
      startCycle();
      return;
    }
  }

  switch (state) {
    case MOVE_TO_LASER:
      handleMoveState(DWELL_AT_LASER);
      break;

    case DWELL_AT_LASER:
      handleDwellState(LASER_DWELL_MS, OUTPUT_MAPPING_DEG, MAPPING_MOVE_MS, MOVE_TO_MAPPING);
      break;

    case MOVE_TO_MAPPING:
      handleMoveState(DWELL_AT_MAPPING);
      break;

    case DWELL_AT_MAPPING:
      handleDwellState(MAP_DWELL_MS, OUTPUT_PICKING_DEG, PICKING_MOVE_MS, MOVE_TO_PICKING);
      break;

    case MOVE_TO_PICKING:
      handleMoveState(CYCLE_COMPLETE);
      break;

    case CYCLE_COMPLETE:
    case ABORTED:
    default:
      break;
  }
}

// ---------------------------
// Motor helpers
// ---------------------------
void motorStop() {
  analogWrite(pwmPin, 0);
  digitalWrite(AIN1Pin, LOW);
  digitalWrite(AIN2Pin, LOW);
  digitalWrite(STBYPin, LOW);
}

void motorBrake() {
  // Short active brake. This helps the mechanism settle before a dwell.
  digitalWrite(STBYPin, HIGH);
  digitalWrite(AIN1Pin, HIGH);
  digitalWrite(AIN2Pin, HIGH);
  analogWrite(pwmPin, 0);
}

void applySignedDuty(int signedDuty) {
  int duty = abs(signedDuty);
  if (duty > 100) duty = 100;

  int pwm255 = (int) ((255.0 * duty) / 100.0 + 0.5);

  digitalWrite(STBYPin, HIGH);

  if (signedDuty > 0) {
    digitalWrite(AIN1Pin, HIGH);
    digitalWrite(AIN2Pin, LOW);
  } else if (signedDuty < 0) {
    digitalWrite(AIN1Pin, LOW);
    digitalWrite(AIN2Pin, HIGH);
  } else {
    digitalWrite(AIN1Pin, LOW);
    digitalWrite(AIN2Pin, LOW);
  }

  analogWrite(pwmPin, pwm255);
}

int dutyForMoveProgress(double progress, double deltaDeg) {
  bool cw = (deltaDeg >= 0.0);

  if (progress < FAST_PHASE_END) {
    return cw ? DUTY_CW_FAST : -DUTY_CCW_FAST;
  } else if (progress < MED_PHASE_END) {
    return cw ? DUTY_CW_MEDIUM : -DUTY_CCW_MEDIUM;
  } else {
    return cw ? DUTY_CW_SLOW : -DUTY_CCW_SLOW;
  }
}

// ---------------------------
// Sequence helpers
// ---------------------------
void startCycle() {
  motorStop();

  // Open-loop version: always define the starting physical position as 0 deg.
  estimatedOutputDeg = OUTPUT_PICKING_DEG;
  moveStartOutputDeg = OUTPUT_PICKING_DEG;
  moveTargetOutputDeg = OUTPUT_PICKING_DEG;
  moveDeltaOutputDeg = 0.0;
  moveDurationMs = 0;
  stateStartMs = millis();
  lastPrintMs = 0;

  Serial.println();
  Serial.println("Cycle starting from PICKING station (0 deg).\nType 'x' + Enter to abort. Type 'r' + Enter to restart after re-homing.");

  startMove(OUTPUT_LASER_DEG, LASER_MOVE_MS, MOVE_TO_LASER);
}

void startMove(double targetOutputDeg, uint32_t plannedMoveMs, SequenceState moveState) {
  moveStartOutputDeg = estimatedOutputDeg;
  moveTargetOutputDeg = targetOutputDeg;
  moveDeltaOutputDeg = moveTargetOutputDeg - moveStartOutputDeg;
  moveDurationMs = plannedMoveMs;
  stateStartMs = millis();
  lastPrintMs = 0;
  state = moveState;

  if (moveState == MOVE_TO_LASER) {
    printStateHeader("Moving to LASER station", targetOutputDeg, plannedMoveMs);
  } else if (moveState == MOVE_TO_MAPPING) {
    printStateHeader("Moving to SURFACE MAPPING station", targetOutputDeg, plannedMoveMs);
  } else {
    printStateHeader("Returning to PICKING station", targetOutputDeg, plannedMoveMs);
  }
}

void enterDwell(SequenceState dwellState) {
  motorBrake();
  delay(120);
  motorStop();

  state = dwellState;
  stateStartMs = millis();
  lastPrintMs = 0;

  // In time-counting mode, the checkpoint error is only an estimate.
  double estimatedErrDeg;
  if (dwellState == DWELL_AT_LASER) {
    estimatedErrDeg = OUTPUT_LASER_DEG - estimatedOutputDeg;
    Serial.print("Reached LASER station. Dwelling for 13.5 s. Estimated Output Err: ");
    Serial.println(estimatedErrDeg, 3);
  } else {
    estimatedErrDeg = OUTPUT_MAPPING_DEG - estimatedOutputDeg;
    Serial.print("Reached SURFACE MAPPING station. Dwelling for 9.0 s. Estimated Output Err: ");
    Serial.println(estimatedErrDeg, 3);
  }
}

void handleMoveState(SequenceState dwellStateAfterMove) {
  uint32_t nowMs = millis();
  uint32_t elapsedMs = nowMs - stateStartMs;

  if (moveDurationMs == 0) {
    estimatedOutputDeg = moveTargetOutputDeg;
  } else {
    double progress = (double) elapsedMs / (double) moveDurationMs;
    if (progress > 1.0) progress = 1.0;
    if (progress < 0.0) progress = 0.0;

    estimatedOutputDeg = moveStartOutputDeg + moveDeltaOutputDeg * progress;

    int dutyCmd = dutyForMoveProgress(progress, moveDeltaOutputDeg);
    applySignedDuty(dutyCmd);

    if (nowMs - lastPrintMs >= SERIAL_OUTPUT_MS) {
      lastPrintMs = nowMs;
      if (state == MOVE_TO_LASER) {
        printMoveUpdate("LASER", elapsedMs, dutyCmd);
      } else if (state == MOVE_TO_MAPPING) {
        printMoveUpdate("MAPPING", elapsedMs, dutyCmd);
      } else {
        printMoveUpdate("PICKING", elapsedMs, dutyCmd);
      }
    }

    if (progress < 1.0) {
      return;
    }
  }

  // Force final estimate to the exact station target once the planned move time ends.
  estimatedOutputDeg = moveTargetOutputDeg;

  if (dwellStateAfterMove == DWELL_AT_LASER || dwellStateAfterMove == DWELL_AT_MAPPING) {
    enterDwell(dwellStateAfterMove);
  } else {
    finishCycle();
  }
}

void handleDwellState(uint32_t dwellMs, double nextTargetDeg, uint32_t nextMoveMs, SequenceState nextMoveState) {
  motorStop();

  uint32_t nowMs = millis();
  uint32_t elapsedMs = nowMs - stateStartMs;

  if (nowMs - lastPrintMs >= 1000) {
    lastPrintMs = nowMs;
    double remainingS = (double) ((elapsedMs >= dwellMs) ? 0 : (dwellMs - elapsedMs)) / 1000.0;
    Serial.print("Dwelling... ");
    Serial.print(remainingS, 1);
    Serial.println(" s remaining");
  }

  if (elapsedMs >= dwellMs) {
    startMove(nextTargetDeg, nextMoveMs, nextMoveState);
  }
}

void finishCycle() {
  motorBrake();
  delay(120);
  motorStop();

  estimatedOutputDeg = OUTPUT_PICKING_DEG;
  state = CYCLE_COMPLETE;

  Serial.println();
  Serial.println("Cycle complete.");
  Serial.print("Estimated final output angle: ");
  Serial.print(estimatedOutputDeg, 3);
  Serial.println(" deg");
  Serial.println("Place lever at the picking station, then type 'r' + Enter to run again.");
}

void abortCycle() {
  motorBrake();
  delay(120);
  motorStop();

  state = ABORTED;

  Serial.println();
  Serial.println("Cycle aborted.");
  Serial.print("Estimated current output angle: ");
  Serial.print(estimatedOutputDeg, 3);
  Serial.println(" deg");
  Serial.println("Manually re-home to the picking station, then type 'r' + Enter to restart.");
}

// ---------------------------
// Serial / printing helpers
// ---------------------------
void printBanner() {
  Serial.println("APSC 200 belt-indexed station controller (TIME-BASED, NO ENCODER)");
  Serial.println("Driver cap diameter  = 4.00 cm");
  Serial.println("Driven cap diameter  = 7.50 cm");
  Serial.println("Assumption: open belt => output rotates same direction as motor.");
  Serial.println("This version estimates angle from time only.");
  Serial.println();
}

void printStateHeader(const char* label, double targetOutputDeg, uint32_t plannedMoveMs) {
  Serial.println();
  Serial.println(label);
  Serial.print("Target output angle (large cap / lever): ");
  Serial.print(targetOutputDeg, 2);
  Serial.println(" deg");
  Serial.print("Planned move time: ");
  Serial.print(plannedMoveMs);
  Serial.println(" ms");
}

void printMoveUpdate(const char* label, uint32_t elapsedMs, int dutyCmd) {
  double estimatedErrDeg = moveTargetOutputDeg - estimatedOutputDeg;

  Serial.print("Moving to ");
  Serial.print(label);
  Serial.print(" | Elapsed: ");
  Serial.print(elapsedMs);
  Serial.print(" / ");
  Serial.print(moveDurationMs);
  Serial.print(" ms | Est. output pos: ");
  Serial.print(estimatedOutputDeg, 2);
  Serial.print(" deg | Est. output err: ");
  Serial.print(estimatedErrDeg, 2);
  Serial.print(" deg | Duty: ");
  Serial.println(dutyCmd);
}

String readLineNonBlocking() {
  static String line = "";

  while (Serial.available() > 0) {
    char c = (char) Serial.read();

    if (c == '\n' || c == '\r') {
      if (line.length() > 0) {
        String out = line;
        line = "";
        return out;
      }
    } else {
      line += c;
    }
  }

  return "";
}
