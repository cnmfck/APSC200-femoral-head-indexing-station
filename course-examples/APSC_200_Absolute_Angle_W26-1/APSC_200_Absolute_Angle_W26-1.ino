/*
  APSC 200
  Raspberry Pi Pico (RP2040) + TB6612FNG motor driver + Quadrature Encoder
  Absolute angle control using ArduPID. Continuous control with cadenced output.

  Author: Jason Harris
  Date:   February 2026

  This code may be used without attribution in APSC 200 or MECHMania 2.0 (W26).
  Otherwise code must be attributed to the author.

  ---------------------------------- Overview ----------------------------------
  For students:
    1) In the Serial Monitor, type an ABSOLUTE angle in degrees (signed) and press Enter.
       Examples: -90, 360, 1080.5. Multiple revolutions are allowed.
    2) The motor rotates to that absolute angle since boot. The position is not reset
       after each entry.
    3) While moving, the code prints Setpoint, Position, and Error every 250 ms.
    4) Type "x" then Enter at any time to brake and stop during motion. When the motor
       reaches the setpoint or is stopped with "x", it brakes briefly, stops, prints the
       final Setpoint, Position, and Error, then waits quietly for the next entry.

  --------------------------------- Key Concepts --------------------------------
    • Absolute Position:
        Angle is computed from cumulative encoder counts since boot in degrees.
        We do not reset the encoder position between user commands.

    • PID Control (ArduPID):
        The encoder angle (degrees) is the PID input.
        The controller output is a signed duty percentage [-100, +100].
        The sign sets direction. The magnitude sets PWM duty.
        A 1.0 degree deadband declares success to prevent chattering.

    • Cadenced Output:
        Control runs every loop for smooth actuation.
        Status prints are throttled to a fixed cadence for readability.

  ------------------------------- Required Libraries -----------------------------
    • RP2040_PWM v1.7.0 by Khoi Hoang
    • PicoEncoder v1.1.1 by Paulo Marques
    • ArduPID v0.2.1 by PowerBroker2
    Install via Arduino Library Manager.

  -------------------------------- Hardware Notes --------------------------------
    • TB6612FNG motor driver:
        - AIN1 and AIN2 control motor direction for one channel.
        - PWMA is the PWM input for that channel.
        - STBY must be HIGH to enable outputs; LOW disables outputs (safe state).
    • Always share ground between the Pico and the motor power supply.
*/

#include "RP2040_PWM.h"   // PWM control for RP2040
#include "PicoEncoder.h"  // Quadrature encoder handling for RP2040

// Silence deprecation warnings from ArduPID's legacy Bxxx debug flags on this core only
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "ArduPID.h"      // PID controller
#pragma GCC diagnostic pop

// ============================================================================
// Pin Assignments (Update to match your wiring)
// ============================================================================
#define encAPin 16    // Encoder channel A (must be consecutive with B)
#define encBPin 17    // Encoder channel B
#define AIN1Pin 12    // TB6612FNG direction input 1
#define AIN2Pin 13    // TB6612FNG direction input 2
#define STBYPin 14    // TB6612FNG standby (LOW = outputs off, HIGH = enabled)
#define pwmPin  15    // TB6612FNG PWMA (motor PWM input)

// ============================================================================
// User Limits & Timing Settings
// ============================================================================
const int MIN_DUTY              = 15;        // Not used as a hard clamp with PID
const int MAX_DUTY              = 100;       // Absolute max allowed duty percentage
const uint32_t SERIAL_OUTPUT_MS = 25;        // Update rate for Serial prints (ms)
static const float PWM_FREQ_HZ  = 1500.0f;   // PWM frequency (1.5 kHz)

// ============================================================================
// Encoder Constants (Change if you use a different encoder/gearbox)
// ============================================================================
// COUNTS_PER_REV is counts per mechanical revolution of the OUTPUT SHAFT.
const float  COUNTS_PER_REV         = 374.22f;                // FIT0521-specific
const double STEPS_PER_REV          = COUNTS_PER_REV * 4.0;   // 4x decoding
const double SUBSTEPS_PER_REV       = STEPS_PER_REV * 64.0;   // library sub-steps
static const double DEG_PER_SUBSTEP = 360.0 / SUBSTEPS_PER_REV;

// ============================================================================
// PID Settings
// ============================================================================
// Setpoint, input, and output variables used by ArduPID
static double setpointDeg = 0.0;  // target angle in degrees (absolute since boot)
static double inputDeg    = 0.0;  // measured angle in degrees
static double outputDuty  = 0.0;  // signed duty command [-100, +100]

// Initial PID gains - tune as needed
static double Kp = 0.4;
static double Ki = 0.002;
static double Kd = 0.05;

// Deadband and braking
static const float POSITION_TOL_DEG = 1.0f; // Stop when |error| <= 1.0 deg
static const uint16_t BRAKE_MS      = 1000; // Short electrical brake duration

// ============================================================================
// Global Objects
// ============================================================================
RP2040_PWM* motorPWM = nullptr;  // Handles PWM output to the motor driver
PicoEncoder  encoder;            // Handles quadrature encoder counting
ArduPID      myPid;              // PID controller

// ============================================================================
// Forward Declarations (keep structure tidy)
// ============================================================================
// Output helpers
void printPrompt();
void printMovingUpdate(double setpointDeg, double angleDeg, double errorDeg);
void printFinalUpdate(double setpointDeg, double angleDeg, double errorDeg);
void printParseError(const String& line);
void printEncoderInitError();

// Motor control helpers
void motorStop();
void motorBrake();
void applySignedDuty(double dutySigned);

// Non-blocking Serial input
String readLineNonBlocking();
bool isNumericSigned(const String& s);

// ============================================================================
// Setup()
// Purpose: Initialize Serial, pins, PWM, encoder, and PID. Show initial prompt.
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } // Wait for USB-CDC on boards like Pico

  // Configure TB6612FNG control pins
  pinMode(AIN1Pin, OUTPUT);
  pinMode(AIN2Pin, OUTPUT);
  pinMode(STBYPin, OUTPUT);

  // Safe default state on power-up: driver disabled, outputs low
  digitalWrite(AIN1Pin, LOW);
  digitalWrite(AIN2Pin, LOW);
  digitalWrite(STBYPin, LOW);

  // Create and initialize the PWM controller at 0% duty
  motorPWM = new RP2040_PWM(pwmPin, PWM_FREQ_HZ, 0.0f);
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);

  // Initialize encoder. Must pass the LOWER of the two consecutive pins.
  if (encoder.begin(encAPin) != 0) {
    printEncoderInitError();
    while (true) { delay(1000); } // Stop here so the error is visible
  }

  // Establish a known reference position at boot
  encoder.resetPosition();

  // Initialize PID with pointers to variables and initial gains
  myPid.begin(&inputDeg, &outputDuty, &setpointDeg, Kp, Ki, Kd);
  myPid.setOutputLimits(-MAX_DUTY, MAX_DUTY);   // Signed duty maps to direction
  myPid.setWindUpLimits(-10, 10);               // Anti-windup in duty units
  myPid.stop();                                 // Disabled until a valid target is entered

  // Show usage instructions to the student
  printPrompt();
}

// ============================================================================
// Loop()
// Purpose: Process user input, run ArduPID-based control, print on cadence.
// ============================================================================
void loop() {
  // Keep encoder state fresh each iteration
  encoder.update();

  // ------------------------- Persistent State Variables ----------------------
  static bool moving = false;                // True while PID is actively driving
  static bool stopMsgShown = false;          // Latch to print the final line once
  static unsigned long lastPrintMs = 0;      // Time of last periodic print (millis)

  // ------------------------- 1) Non-blocking User Input ---------------------
  String line = readLineNonBlocking();

  if (line.length() > 0) {
    line.trim();

    // --- A) Stop command: 'x' ---
    if (line.equalsIgnoreCase("x")) {
      // Brake briefly, then stop with outputs disabled
      motorBrake();
      delay(BRAKE_MS);
      encoder.update();  // <-- added: capture post-brake true counts
      motorStop();

      // Freeze PID and clear history
      myPid.stop();
      myPid.reset();

      // Show final Setpoint, Position, and Error once
      inputDeg = ((double)encoder.position) * DEG_PER_SUBSTEP;
      const double err = setpointDeg - inputDeg;
      if (!stopMsgShown) {
        printFinalUpdate(setpointDeg, inputDeg, err);
        stopMsgShown = true;
      }

      // Become idle and prompt again
      moving = false;
      printPrompt();
    }
    // --- B) Numeric absolute angle input ---
    else {
      if (isNumericSigned(line)) {
        // Update target angle in degrees, absolute since boot
        setpointDeg = (double)line.toFloat();

        // New command - allow a final message for this run
        stopMsgShown = false;

        // Evaluate current error
        inputDeg = ((double)encoder.position) * DEG_PER_SUBSTEP;
        const double err = setpointDeg - inputDeg;

        if (fabs(err) <= POSITION_TOL_DEG) {
          // Already within deadband: brake to hold, stop, print once, then quiet
          motorBrake();
          delay(BRAKE_MS);
          encoder.update();  // <-- added: capture post-brake true counts
          motorStop();

          myPid.stop();
          myPid.reset();

          // Recompute using the fresh position
          inputDeg = ((double)encoder.position) * DEG_PER_SUBSTEP;
          const double err2 = setpointDeg - inputDeg;
          printFinalUpdate(setpointDeg, inputDeg, err2);
          moving = false;
          printPrompt();
        } else {
          // Start PID for active motion
          myPid.reset();  // fresh I and D terms
          myPid.start();

          moving = true;
          lastPrintMs = millis(); // print soon after starting

          // Inform the user that motion is underway and how to stop
          Serial.println("Moving... Type 'x' + Enter at any time to brake and stop.");
        }
      } else {
        // Not 'x' and not a valid signed number → parse error with guidance
        printParseError(line);
      }
    }
  }

  // ------------------- 2) Continuous Control While Moving -------------------
  if (moving) {
    // Update measurement angle in degrees
    inputDeg = ((double)encoder.position) * DEG_PER_SUBSTEP;
    const double err = setpointDeg - inputDeg;

    // Success condition: inside deadband
    if (fabs(err) <= POSITION_TOL_DEG) {
      motorBrake();
      delay(BRAKE_MS);
      encoder.update();  // <-- added: capture post-brake true counts
      motorStop();

      myPid.stop();
      myPid.reset();

      // Recompute after brake for true final values
      inputDeg = ((double)encoder.position) * DEG_PER_SUBSTEP;
      const double err2 = setpointDeg - inputDeg;
      printFinalUpdate(setpointDeg, inputDeg, err2);
      moving = false;
      stopMsgShown = true;
      printPrompt();
    } else {
      // Compute PID and apply signed duty [-100, +100]
      myPid.compute();
      applySignedDuty(outputDuty);
    }

    // ----------------------- 3) Periodic Status Prints ----------------------
    const unsigned long nowMs = millis();
    if (moving && (nowMs - lastPrintMs >= SERIAL_OUTPUT_MS)) {
      lastPrintMs = nowMs;

      // Print Setpoint, Position, and Error at a fixed cadence
      printMovingUpdate(setpointDeg, inputDeg, (setpointDeg - inputDeg));
    }
  }
}

// ============================================================================
// Output Helpers
// Centralize ALL Serial prints for tidy control flow in loop().
// ============================================================================

/*
  Function: printPrompt
  Purpose : Show instructions and the input prompt to the user.
*/
void printPrompt() {
  Serial.println();
  Serial.println("Enter ABSOLUTE angle in degrees (negative allowed, decimals ok).");
  Serial.println("Examples: -90, 360, 1080.5");
  Serial.println("Tip: While the motor is MOVING, type 'x' + Enter at any time to BRAKE & STOP.");
  Serial.print("> ");
  Serial.flush();
}

/*
  Function: printMovingUpdate
  Purpose : Print the periodic line while moving showing setpoint, position, and error.
*/
void printMovingUpdate(double setpointDeg, double angleDeg, double errorDeg) {
  Serial.print("Setpoint: ");
  Serial.print(setpointDeg, 2);
  Serial.print(" deg | Position: ");
  Serial.print(angleDeg, 2);
  Serial.print(" deg | Error: ");
  Serial.print(errorDeg, 2);
  Serial.println(" deg");
}

/*
  Function: printFinalUpdate
  Purpose : Report the final setpoint, position, and error when a run stops.
*/
void printFinalUpdate(double setpointDeg, double angleDeg, double errorDeg) {
  Serial.print("\nFinal → Setpoint: ");
  Serial.print(setpointDeg, 3);
  Serial.print(" deg | Position: ");
  Serial.print(angleDeg, 3);
  Serial.print(" deg | Error: ");
  Serial.print(errorDeg, 3);
  Serial.println(" deg");
}

/*
  Function: printParseError
  Purpose : Explain that the input was not a valid signed number or 'x'.
*/
void printParseError(const String& line) {
  Serial.print("Invalid input: '");
  Serial.print(line);
  Serial.println("'. Enter a signed number (e.g., -90, 360, 1080.5) or 'x' to stop.");
  Serial.print("> ");
}

/*
  Function: printEncoderInitError
  Purpose : Report encoder initialization failure and provide guidance.
*/
void printEncoderInitError() {
  Serial.println("ERROR: Encoder init failed.");
  Serial.println("Ensure A and B pins are consecutive, correctly wired, and free.");
}

// ============================================================================
// Motor Control Helpers
// ============================================================================

/*
  Function: motorStop
  Purpose : Safe, no-torque stop. Sets duty to 0% and disables H-bridge (STBY LOW).
*/
void motorStop() {
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);
  digitalWrite(AIN1Pin, LOW);
  digitalWrite(AIN2Pin, LOW);
  digitalWrite(STBYPin, LOW);  // Outputs disabled (standby)
}

/*
  Function: motorBrake
  Purpose : Short-brake with both inputs HIGH at 0% duty.
            This increases damping and slows the motor more quickly than coasting.
*/
void motorBrake() {
  digitalWrite(STBYPin, HIGH);
  digitalWrite(AIN1Pin, HIGH);
  digitalWrite(AIN2Pin, HIGH);
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);
}

/*
  Function: applySignedDuty
  Purpose : Apply a signed duty command from PID output.
            Positive = forward (AIN1 HIGH, AIN2 LOW), Negative = reverse.
*/
void applySignedDuty(double dutySigned) {
  // Clamp to safe range [-100, 100]
  if (dutySigned >  100.0) dutySigned =  100.0;
  if (dutySigned < -100.0) dutySigned = -100.0;

  double mag = fabs(dutySigned);
  uint8_t duty = (uint8_t)(mag + 0.5); // nearest integer percentage

  if (duty == 0) {
    // Coast in current direction with outputs enabled for quick re-apply
    digitalWrite(STBYPin, HIGH);
    motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);
    return;
  }

  if (dutySigned >= 0.0) {
    // Forward direction on TB6612FNG: AIN1 HIGH, AIN2 LOW
    digitalWrite(STBYPin, HIGH);
    digitalWrite(AIN1Pin, HIGH);
    digitalWrite(AIN2Pin, LOW);
    motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, (float)duty);
  } else {
    // Reverse direction: AIN1 LOW, AIN2 HIGH
    digitalWrite(STBYPin, HIGH);
    digitalWrite(AIN1Pin, LOW);
    digitalWrite(AIN2Pin, HIGH);
    motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, (float)duty);
  }
}

// ============================================================================
// Non-blocking Serial Line Reader and Validator
// ============================================================================

/*
  Function: readLineNonBlocking
  Purpose : Read a full line without blocking loop execution.
            Returns an empty String if a full line has not arrived yet.
            Handles CRLF by ignoring '\r' and terminating on '\n'.
*/
String readLineNonBlocking() {
  static String buf = "";  // Persistent buffer that accumulates characters

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue; // Ignore carriage returns (Windows CRLF)
    if (c == '\n') {
      // We have a full line; return trimmed result and clear buffer
      String line = buf;
      buf = "";
      line.trim();
      return line;
    }

    // Accumulate characters until newline is received
    buf += c;
  }

  // No complete line yet
  return "";
}

/*
  Function: isNumericSigned
  Purpose : Validate a signed decimal number string: optional +/-,
            digits, optional single '.', at least one digit.
*/
bool isNumericSigned(const String& s) {
  if (s.length() == 0) return false;
  size_t i = 0;
  if (s[0] == '+' || s[0] == '-') {
    if (s.length() == 1) return false;
    i = 1;
  }
  bool hasDigit = false;
  bool hasDot = false;
  for (; i < (size_t)s.length(); i++) {
    char c = s[i];
    if (c >= '0' && c <= '9') {
      hasDigit = true;
    } else if (c == '.') {
      if (hasDot) return false;
      hasDot = true;
    } else {
      return false;
    }
  }
  return hasDigit;
}