/*
  APSC 200
  Indexed station sequence for a belt-driven cap system.

  This sketch makes the motor rotate a SMALL cap. A rubber band (open belt)
  connects that small cap to a LARGER cap. A fixed lever is attached to the
  larger cap, so the lever is the final output that performs the station-to-
  station motion.

  The required output motion is:
    1) Start at the picking station                  ->   0 deg
    2) Rotate 108 deg clockwise to laser station     -> 108 deg
    3) Wait 13.5 s at laser station
    4) Rotate 198 deg counterclockwise to mapping    -> -90 deg net angle
    5) Wait 9.0 s at mapping station
    6) Rotate 90 deg clockwise back to picking       ->   0 deg

  Because the motor is NOT directly attached to the final lever, the code must
  convert each desired OUTPUT / lever angle into the equivalent MOTOR angle.


  MECHANISM GEOMETRY

    Small cap diameter (motor side) = 4.0 cm
    Large cap diameter (lever side) = 7.5 cm

    For an open belt, both caps rotate in the SAME direction.

    With no slip:
        theta_motor * D_small = theta_output * D_large

        theta_motor = theta_output * (D_large / D_small)
                    = theta_output * (7.5 / 4.0)
                    = theta_output * 1.875

    So the output angles become these motor targets:
        Output   0.0 deg  -> Motor    0.00 deg
        Output 108.0 deg  -> Motor  202.50 deg
        Output -90.0 deg  -> Motor -168.75 deg


  IMPORTANT STARTING ASSUMPTION

  Before power-up or before typing 'r' to run again, physically place the lever
  at the PICKING STATION. The encoder is reset there, so the code assumes that
  physical position is exactly 0 degrees


  SERIAL COMMANDS

    x + Enter  -> abort the current cycle immediately
    r + Enter  -> restart the cycle after you manually place the lever back at
                  the picking station
*/

#include "RP2040_PWM.h"
#include "PicoEncoder.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "ArduPID.h"
#pragma GCC diagnostic pop


// Pin Assignments
#define encAPin 16
#define encBPin 17
#define AIN1Pin 12
#define AIN2Pin 13
#define STBYPin 14
#define pwmPin  15


// Motor / PWM Settings
// MIN_EFFECTIVE_DUTY: small extra bias so the motor can overcome static
// friction instead of getting stuck near the target
// Anti-stall tuning:
// - raise the minimum effective duty so the motor keeps enough torque
//   when it is close to the target but still needs to keep moving
// - keep full 100% duty available for long moves under heavier load
const int MIN_EFFECTIVE_DUTY      = 45;
const int MAX_DUTY                = 100;
const uint32_t SERIAL_OUTPUT_MS   = 250;
static const float PWM_FREQ_HZ    = 1500.0f;
static const uint16_t BRAKE_MS    = 250;

// Near the target, allow a smaller floor so the mechanism can settle instead
// of always hitting the station at the full minimum duty.
static const double NEAR_TARGET_OUTPUT_ERR_DEG = 2.0;
static const int    NEAR_TARGET_MIN_DUTY       = 32;

// If the encoder barely changes for a while even though error is still large,
// treat it as a stall and apply a short stronger push in the needed direction.
static const uint32_t STALL_CHECK_MS           = 300;
static const double   STALL_PROGRESS_MOTOR_DEG = 2.0;
static const double   STALL_ERROR_OUTPUT_DEG   = 2.0;
static const int      STALL_KICK_DUTY          = 65;


// Encoder Conversion Constants
// The encoder library reports position in encoder counts. The example code
// further accounts for quadrature decoding and internal substeps so that the
// position can be converted to a motor angle in degrees.
const float  COUNTS_PER_REV         = 374.22f;
const double STEPS_PER_REV          = COUNTS_PER_REV * 4.0;
const double SUBSTEPS_PER_REV       = STEPS_PER_REV * 64.0;
static const double DEG_PER_SUBSTEP = 360.0 / SUBSTEPS_PER_REV;


// Belt / Cap Geometry
// DRIVER = small cap mounted on the motor shaft
// DRIVEN = large cap carrying the fixed output lever
static const double DRIVER_DIAMETER_CM = 4.0;
static const double DRIVEN_DIAMETER_CM = 7.5;

// Open belt => motor and output rotate in the same angular direction.
// If you ever rebuild the mechanism and it rotates the opposite way, change
// this from +1.0 to -1.0.
static const double MOTOR_POSITIVE_GIVES_OUTPUT_CW = +1.0;

// For every 1 degree of OUTPUT rotation, the MOTOR must rotate 1.875 degrees.
static const double MOTOR_DEG_PER_OUTPUT_DEG = DRIVEN_DIAMETER_CM / DRIVER_DIAMETER_CM;


// Required Output / Lever Absolute Angles
// Clockwise is taken as positive here.
//
// Picking station: 0 deg
// Laser station:   +108 deg from picking
// Mapping station: +108 deg then -198 deg = -90 deg net from picking
static const double OUTPUT_PICKING_DEG =   0.0;
static const double OUTPUT_LASER_DEG   = 108.0;
static const double OUTPUT_MAPPING_DEG = -90.0;

// Station dwell times from the requirement list.
static const uint32_t LASER_DWELL_MS   = 13500UL;
static const uint32_t MAP_DWELL_MS     =  9000UL;

// Tolerance is defined at the output lever, then converted to the equivalent
// allowed error at the motor side.
static const double OUTPUT_TOL_DEG = 1.0;
static const double MOTOR_TOL_DEG  = OUTPUT_TOL_DEG * MOTOR_DEG_PER_OUTPUT_DEG;


// PID Settings
// setpointDeg = where the motor should go
// inputDeg    = where the motor currently is
// outputDuty  = control output sent to the motor driver (signed duty)

static double setpointDeg = 0.0;
static double inputDeg    = 0.0;
static double outputDuty  = 0.0;

// Slightly stronger P and I action than the original file so the controller
// keeps pushing through real-system friction and belt/load variation.
static double Kp = 0.65;
static double Ki = 0.010;
static double Kd = 0.03;


// Global Objects
RP2040_PWM* motorPWM = nullptr;
PicoEncoder  encoder;
ArduPID      myPid;


// Sequence State Machine
// The program moves through these named states one by one.
// This makes the station sequence easy to follow and debug.
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
static uint32_t stateStartMs = 0;   // when the current state began
static uint32_t lastPrintMs  = 0;   // used to limit serial print frequency
static bool holdingPosition  = false;

// Extra motion-health tracking for anti-stall behavior.
static uint32_t lastMotionCheckMs = 0;
static double   lastMotionCheckDeg = 0.0;


// Forward Declarations
// These tell the compiler which functions exist later in the file.
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
void printMoveUpdate(const char* label);
void printCompleteMessage();
void printAbortMessage();
void printEncoderInitError();
int computeAntiStallDuty(double rawDuty, double motorErrDeg, double outputErrDeg);

String readLineNonBlocking();

double outputToMotorDeg(double outputAngleDegCW);
double motorToOutputDeg(double motorAngleDeg);


// setup()
// Runs ONCE at startup
//
//   1) start Serial communication
//   2) configure motor-driver pins
//   3) create PWM output on the same pin/frequency as the example
//   4) initialize encoder
//   5) initialize the PID controller
//   6) print mechanism info and start the first cycle automatically
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // Set H-bridge control pins as outputs
  pinMode(AIN1Pin, OUTPUT);
  pinMode(AIN2Pin, OUTPUT);
  pinMode(STBYPin, OUTPUT);

  // Start with the motor driver disabled / motor off
  digitalWrite(AIN1Pin, LOW);
  digitalWrite(AIN2Pin, LOW);
  digitalWrite(STBYPin, LOW);

  // Create the PWM object and explicitly initialize it to 0% duty
  motorPWM = new RP2040_PWM(pwmPin, PWM_FREQ_HZ, 0.0f);
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);

  // Start the quadrature encoder
  // If it fails, stop here so the mechanism does not move blindly
  if (encoder.begin(encAPin) != 0) {
    printEncoderInitError();
    while (true) { delay(1000); }
  }

  // The mechanism is assumed to be physically placed at the picking station
  // so we define the current encoder position as zero
  encoder.resetPosition();

  // Connect PID input/output/setpoint variables
  myPid.begin(&inputDeg, &outputDuty, &setpointDeg, Kp, Ki, Kd);
  myPid.setOutputLimits(-MAX_DUTY, MAX_DUTY);
  myPid.setWindUpLimits(-10, 10);
  myPid.stop();

  printBanner();
  startCycle();
}


// loop()
// Runs repeatedly forever after setup()
//
//   1) update encoder reading
//   2) convert encoder position into motor angle
//   3) check Serial for user commands (abort / restart)
//   4) execute the correct behavior for the current state in the sequence
void loop() {
  // Refresh encoder position and convert it to motor angle in degrees
  encoder.update();
  inputDeg = ((double)encoder.position) * DEG_PER_SUBSTEP;

  // Read any complete line from Serial without blocking program execution
  String line = readLineNonBlocking();
  if (line.length() > 0) {
    line.trim();

    // Abort command: stop the current cycle immediately
    if (line.equalsIgnoreCase("x")) {
      abortCycle();
      return;
    }

    // Restart command
    // user must first place the lever back at picking station
    if (line.equalsIgnoreCase("r")) {
      startCycle();
      return;
    }
  }

  // State machine
  // choose the right action based on where we are in the cycle
  switch (state) {
    case MOVE_TO_LASER:
      handleMoveState(DWELL_AT_LASER);
      break;

    case DWELL_AT_LASER:
      handleDwellState(LASER_DWELL_MS, OUTPUT_MAPPING_DEG, MOVE_TO_MAPPING);
      break;

    case MOVE_TO_MAPPING:
      handleMoveState(DWELL_AT_MAPPING);
      break;

    case DWELL_AT_MAPPING:
      handleDwellState(MAP_DWELL_MS, OUTPUT_PICKING_DEG, MOVE_TO_PICKING);
      break;

    case MOVE_TO_PICKING:
      handleMoveState(CYCLE_COMPLETE);
      break;

    case CYCLE_COMPLETE:
    case ABORTED:
    default:
      // Do nothing here. The mechanism is idle until the user restarts it
      break;
  }
}


// Geometry Helper
// Convert the desired OUTPUT / lever angle into the required MOTOR angle
// The motor and output do not have a 1:1 connection. 
// The belt ratio means the motor must rotate more than the output lever
double outputToMotorDeg(double outputAngleDegCW) {
  return MOTOR_POSITIVE_GIVES_OUTPUT_CW * MOTOR_DEG_PER_OUTPUT_DEG * outputAngleDegCW;
}


// Geometry Helper
//   Convert the measured MOTOR angle back into an OUTPUT / lever angle.
//   This makes Serial messages easier to interpret, because the user usually cares about the lever angle at the stations, not just the raw motor angle
double motorToOutputDeg(double motorAngleDeg) {
  return motorAngleDeg / (MOTOR_POSITIVE_GIVES_OUTPUT_CW * MOTOR_DEG_PER_OUTPUT_DEG);
}


// Sequence Helper: startCycle()
//   Reinitialize the entire motion sequence from the picking station.

// stops the motor
// resets encoder zero at the current physical lever position
// clears PID state
// resets timers and print timing
// commands the first move: picking -> laser
void startCycle() {
  motorStop();

  // update once, then define the current physical position as zero
  encoder.update();
  encoder.resetPosition();

  // Reset all main control variables
  inputDeg = 0.0;
  setpointDeg = 0.0;
  outputDuty = 0.0;

  // Clear any old PID memory from the previous run
  myPid.stop();
  myPid.reset();

  // Reset timers and flags for the new cycle
  stateStartMs = millis();
  lastPrintMs  = 0;
  holdingPosition = false;
  lastMotionCheckMs = millis();
  lastMotionCheckDeg = inputDeg;

  Serial.println();
  Serial.println("Cycle starting from PICKING station (0 deg).");
  Serial.println("Type 'x' + Enter to abort. Type 'r' + Enter to restart after re-homing.");

  // First required move in the sequence
  beginMoveToOutputAngle(OUTPUT_LASER_DEG);
  state = MOVE_TO_LASER;
}


// Sequence Helper: beginMoveToOutputAngle()
// Start a move toward a new absolute OUTPUT station angle

// cnverts the requested output angle into a motor setpoint
// resets and starts the PID loop
// resets state timers used for prints / timing
// prints a readable header showing the target station

void beginMoveToOutputAngle(double outputAngleDegCW) {
  setpointDeg = outputToMotorDeg(outputAngleDegCW);

  myPid.reset();
  myPid.start();

  holdingPosition = false;
  stateStartMs = millis();
  lastPrintMs  = 0;
  lastMotionCheckMs = millis();
  lastMotionCheckDeg = inputDeg;

  if (outputAngleDegCW == OUTPUT_LASER_DEG) {
    printStateHeader("Moving to LASER station", outputAngleDegCW);
  } else if (outputAngleDegCW == OUTPUT_MAPPING_DEG) {
    printStateHeader("Moving to SURFACE MAPPING station", outputAngleDegCW);
  } else {
    printStateHeader("Returning to PICKING station", outputAngleDegCW);
  }
}


// Sequence Helper
//Stop motion and start a timed dwell at a station
// actively brakes briefly to settle the mechanism
// stops the motor completely
// stops / resets PID because no new move is happening during dwell
//  changes state to the appropriate dwell state
// starts the dwell timer

void enterDwell(SequenceState dwellState) {
  // short brake helps the mechanism settle before waiting
  motorBrake();
  delay(BRAKE_MS);

  // Refresh encoder one more time after braking
  encoder.update();

  // Turn motor fully off during the dwell.
  motorStop();
  myPid.stop();
  myPid.reset();

  holdingPosition = true;
  state = dwellState;
  stateStartMs = millis();
  lastPrintMs = 0;

  if (dwellState == DWELL_AT_LASER) {
    Serial.println("Reached LASER station. Dwelling for 13.5 s.");
  } else {
    Serial.println("Reached SURFACE MAPPING station. Dwelling for 9.0 s.");
  }
}


// Sequence Helper
//   Execute one control step while the mechanism is moving toward a station

// computes current angle error
// checks whether target is reached within tolerance
// if not there yet, runs PID and applies motor duty
// periodically prints motion progress to Serial
void handleMoveState(SequenceState completedMoveNextState) {
  // Error between where we want the motor to be and where it is now
  const double motorErrDeg  = setpointDeg - inputDeg;
  const double outputErrDeg = motorToOutputDeg(motorErrDeg);

  // If the motor-angle error is within the equivalent output tolerance, the move is considered complete
  if (fabs(motorErrDeg) <= MOTOR_TOL_DEG) {
    if (completedMoveNextState == DWELL_AT_LASER || completedMoveNextState == DWELL_AT_MAPPING) {
      enterDwell(completedMoveNextState);
    } else {
      finishCycle();
    }
    return;
  }

  // Update the PID output based on current input and setpoint
  myPid.compute();

  // Convert the raw PID request into a more stall-resistant duty command.
  // This adds:
  //   1) a duty floor to overcome static friction
  //   2) a smaller near-target floor to reduce overshoot
  //   3) a short stronger push if the mechanism has stopped progressing
  double dutyCmd = computeAntiStallDuty(outputDuty, motorErrDeg, outputErrDeg);

  // Send the signed duty command to the H-bridge + PWM output
  applySignedDuty(dutyCmd);

  // Print status at a limited rate so the Serial monitor stays readable
  uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= SERIAL_OUTPUT_MS) {
    lastPrintMs = nowMs;

    // outputErrDeg is currently not printed directly, but keeping it here
    // makes debugging easier if you want to expand the printout later.
    (void)outputErrDeg;

    if (state == MOVE_TO_LASER) {
      printMoveUpdate("LASER");
    } else if (state == MOVE_TO_MAPPING) {
      printMoveUpdate("MAPPING");
    } else {
      printMoveUpdate("PICKING");
    }
  }
}


// Sequence Helper: handleDwellState()
//   Wait at a station for the required processing time

// checks how long we have already waited
// prints remaining dwell time once per second
// when the dwell is finished, starts the next move
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

  // When the wait time is over, begin the next commanded move
  if (elapsedMs >= dwellMs) {
    state = nextMoveState;
    beginMoveToOutputAngle(nextOutputAngleDegCW);
  }
}


// Sequence Helper
//   Cleanly stop the mechanism after the final return to picking station

// briefly brakes
// stops the motor
// stops / resets PID
// switches state to CYCLE_COMPLETE
// prints final status message

void finishCycle() {
  motorBrake();
  delay(BRAKE_MS);
  encoder.update();
  motorStop();
  myPid.stop();
  myPid.reset();

  state = CYCLE_COMPLETE;
  printCompleteMessage();
}


// Sequence Helper
// Emergency-style user stop when 'x' is typed in Serial

//  brakes the motor briefly
// stops motor output
// stops / resets PID
// changes state to ABORTED
// tells the user to manually re-home before restarting
void abortCycle() {
  motorBrake();
  delay(BRAKE_MS);
  encoder.update();
  motorStop();
  myPid.stop();
  myPid.reset();

  state = ABORTED;
  printAbortMessage();
}


// Output Helper
//  Print the key geometry and assumption information once at startup
void printBanner() {
  Serial.println();
  Serial.println("APSC 200 belt-indexed station controller");
  Serial.print("Driver cap diameter  = ");
  Serial.print(DRIVER_DIAMETER_CM, 2);
  Serial.println(" cm");
  Serial.print("Driven cap diameter  = ");
  Serial.print(DRIVEN_DIAMETER_CM, 2);
  Serial.println(" cm");
  Serial.print("Motor deg / output deg = ");
  Serial.println(MOTOR_DEG_PER_OUTPUT_DEG, 4);
  Serial.println("Assumption: open belt => output rotates same direction as motor.");
}

// Output Helper
//   Print the new target station whenever a move begins
void printStateHeader(const char* label, double outputAngleDegCW) {
  Serial.println();
  Serial.println(label);
  Serial.print("Target output angle (large cap / lever): ");
  Serial.print(outputAngleDegCW, 2);
  Serial.println(" deg");
  Serial.print("Target motor angle: ");
  Serial.print(outputToMotorDeg(outputAngleDegCW), 2);
  Serial.println(" deg");
}

//Print live position/error information while moving
void printMoveUpdate(const char* label) {
  const double currentOutputDeg = motorToOutputDeg(inputDeg);
  const double targetOutputDeg  = motorToOutputDeg(setpointDeg);
  const double errorOutputDeg   = targetOutputDeg - currentOutputDeg;

  Serial.print("Moving to ");
  Serial.print(label);
  Serial.print(" | Output pos: ");
  Serial.print(currentOutputDeg, 2);
  Serial.print(" deg | Output err: ");
  Serial.print(errorOutputDeg, 2);
  Serial.print(" deg | Motor pos: ");
  Serial.print(inputDeg, 2);
  Serial.print(" deg | Duty: ");
  Serial.println(outputDuty, 2);
}

// Tell the user the cycle is done and how to run it again
void printCompleteMessage() {
  const double finalOutputDeg = motorToOutputDeg(inputDeg);
  Serial.println();
  Serial.println("Cycle complete.");
  Serial.print("Final output angle: ");
  Serial.print(finalOutputDeg, 3);
  Serial.println(" deg");
  Serial.println("Place lever at the picking station, then type 'r' + Enter to run again.");
}

//Tell the cycle was aborted and that manual re-homing is needed
void printAbortMessage() {
  const double abortOutputDeg = motorToOutputDeg(inputDeg);
  Serial.println();
  Serial.println("Cycle aborted.");
  Serial.print("Current output angle: ");
  Serial.print(abortOutputDeg, 3);
  Serial.println(" deg");
  Serial.println("Manually re-home to the picking station, then type 'r' + Enter to restart.");
}

// Print a helpful message if encoder initialization fails.
void printEncoderInitError() {
  Serial.println("ERROR: Encoder init failed.");
  Serial.println("Ensure A and B pins are consecutive, correctly wired, and free.");
}


// Convert the raw PID output into a duty command that is less likely to stall.
//
// Strategy:
//   - If PID asks for a tiny nonzero duty while error is still meaningful,
//     raise it to a minimum effective duty.
//   - When the mechanism is very close to the target, use a smaller floor so
//     the motion can still settle gently.
//   - If encoder motion has been almost frozen for a short time while the error
//     is still significant, apply a short stronger push in the needed direction.
int computeAntiStallDuty(double rawDuty, double motorErrDeg, double outputErrDeg) {
  double dutyCmd = rawDuty;

  const double absOutErr = fabs(outputErrDeg);
  const double absMotorErr = fabs(motorErrDeg);

  // Normal anti-stall duty floor.
  if (fabs(dutyCmd) > 0.0) {
    const int floorDuty = (absOutErr <= NEAR_TARGET_OUTPUT_ERR_DEG)
                        ? NEAR_TARGET_MIN_DUTY
                        : MIN_EFFECTIVE_DUTY;

    if (fabs(dutyCmd) < floorDuty) {
      dutyCmd = (dutyCmd > 0.0) ? floorDuty : -floorDuty;
    }
  }

  // Stall detection: if position barely changed over the last check interval
  // but error is still not small, apply a stronger kick in the needed direction.
  uint32_t nowMs = millis();
  if (nowMs - lastMotionCheckMs >= STALL_CHECK_MS) {
    const double movedDeg = fabs(inputDeg - lastMotionCheckDeg);

    if (absOutErr > STALL_ERROR_OUTPUT_DEG && movedDeg < STALL_PROGRESS_MOTOR_DEG) {
      dutyCmd = (motorErrDeg >= 0.0) ? STALL_KICK_DUTY : -STALL_KICK_DUTY;
    }

    lastMotionCheckMs = nowMs;
    lastMotionCheckDeg = inputDeg;
  }

  // Final clamp to valid range.
  if (dutyCmd >  MAX_DUTY) dutyCmd =  MAX_DUTY;
  if (dutyCmd < -MAX_DUTY) dutyCmd = -MAX_DUTY;

  return (int)((dutyCmd >= 0.0) ? (dutyCmd + 0.5) : (dutyCmd - 0.5));
}


// Motor Helper
//   Fully disable motor drive
// PWM duty = 0
// both direction pins LOW
// standby LOW
// This is the relaxed / non-braking stop
void motorStop() {
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);
  digitalWrite(AIN1Pin, LOW);
  digitalWrite(AIN2Pin, LOW);
  digitalWrite(STBYPin, LOW);
}


// applys an electrical brake through the H-bridge.

//standby HIGH so the driver is active
//both direction pins HIGH to short the motor terminals via the driver
//PWM = 0
// This is used briefly to settle the mechanism before stopping

void motorBrake() {
  digitalWrite(STBYPin, HIGH);
  digitalWrite(AIN1Pin, HIGH);
  digitalWrite(AIN2Pin, HIGH);
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);
}


//Send a signed duty command to the motor.
// INPUT:
// positive duty -> one direction
// negative duty -> the opposite direction

//clamps the requested duty to [-100, 100]
// chooses H-bridge direction pins from the sign
// lets PWM magnitude from the absolute value

void applySignedDuty(double dutySigned) {
  // Keep duty within valid percentage limits.
  if (dutySigned >  100.0) dutySigned =  100.0;
  if (dutySigned < -100.0) dutySigned = -100.0;

  // Convert magnitude to an integer duty value for the PWM library.
  uint8_t duty = (uint8_t)(fabs(dutySigned) + 0.5);

  // Special case: zero duty means no active drive
  if (duty == 0) {
    digitalWrite(STBYPin, HIGH);
    motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);
    return;
  }

  // Enable the driver
  digitalWrite(STBYPin, HIGH);

  // Select motor direction from the sign of the requested duty
  if (dutySigned >= 0.0) {
    digitalWrite(AIN1Pin, HIGH);
    digitalWrite(AIN2Pin, LOW);
  } else {
    digitalWrite(AIN1Pin, LOW);
    digitalWrite(AIN2Pin, HIGH);
  }

  // apply the PWM magnitude
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, (float)duty);
}


// Serial Helper

//Read a full line from the Serial monitor without pausing the rest of the control loop
//The mechanism can keep moving and updating while still allowing the user to type commands such as x or r

//characters are added to a static buffer
//carriage returns are ignored
//when newline arrives, the completed line is returned
//if no complete line is ready yet, return an empty string

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
