/*
  APSC 200
  Raspberry Pi Pico (RP2040) + TB6612FNG motor driver + Quadrature Encoder
  Open loop motor control using duty cycle input from Serial.
  Measures and prints RPM from time-based differences in encoder counts.

  Author: Jason Harris
  Date:   February 2026

  This code may be used without attribution in APSC 200 or MECHMania 2.0 (W26).
  Otherwise code must be attributed to the author.

  ---------------------------------- Overview ----------------------------------
  For students:
    1) In the Serial Monitor, type a duty cycle (an integer) and press Enter.
    2) The motor runs forward at that PWM duty percentage.
    3) While running, the code periodically prints: Duty, RPM, and Revolutions.
    4) Type "x" then Enter to brake and stop. The code prints the average RPM and
       total revolutions for that run, then resets all run totals.

  --------------------------------- Key Concepts --------------------------------
    • PWM (Pulse Width Modulation):
        Digital output switches ON/OFF quickly. The fraction of time it is ON
        is the "duty cycle" (0 to 100%). Higher duty ≈ higher average voltage.

    • Duty Cycle (0–100%):
        The percentage of the PWM period that is HIGH (ON). In open loop, higher
        duty generally means higher speed, but load torque and friction matter.

    • Quadrature Encoder:
        Two digital signals (A and B) that are 90° out of phase. By watching
        their transitions, we can count steps and determine direction.

    • RPM by Timestamp:
        RPM is computed from the change in encoder counts (Δcounts) divided by
        the elapsed time (Δt). We do NOT use encoder.speed from the library.

  ------------------------------- Required Libraries -----------------------------
    • RP2040_PWM v1.7.0 by Khoi Hoang
    • PicoEncoder v1.1.1 by Paulo <arques>
    Install via Arduino Library Manager or PlatformIO.

  -------------------------------- Hardware Notes --------------------------------
    • TB6612FNG motor driver:
        - AIN1 and AIN2 control motor direction for one channel.
        - PWMA is the PWM input for that channel.
        - STBY must be HIGH to enable outputs; LOW disables outputs (safe state).
    • Always share ground between the Pico and the motor power supply.
*/

#include "RP2040_PWM.h"   // PWM control for RP2040
#include "PicoEncoder.h"  // Quadrature encoder handling for RP2040

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
const int MIN_DUTY              = 15;        // Minimum safe/functional duty for lab motors
const int MAX_DUTY              = 100;       // Absolute maximum allowed
const uint32_t RPM_SAMPLE_MS    = 250;       // Update rate for Serial prints (ms)
static const float PWM_FREQ_HZ  = 1500.0f;   // PWM frequency (1.5 kHz)

// ============================================================================
// Run Limits
// ============================================================================
// Set to -1 to disable. If set to a positive number, the motor will brake when
// revolutions >= MAX_REVOLUTIONS and prompt the user to press 'x' to finalize.
const double MAX_REVOLUTIONS = -1.0;

// ============================================================================
// Encoder Constants (Change if you use a different encoder/gearbox)
// ============================================================================
// COUNTS_PER_REV is counts per mechanical revolution of the OUTPUT SHAFT.
const float  COUNTS_PER_REV     = 374.22f;             // FIT0521-specific
const double STEPS_PER_REV      = COUNTS_PER_REV * 4.0;   // 4x decoding
const double SUBSTEPS_PER_REV   = STEPS_PER_REV * 64.0;   // library sub-steps

// ============================================================================
// Global Objects
// ============================================================================
RP2040_PWM* motorPWM = nullptr;  // Handles PWM output to the motor driver
PicoEncoder  encoder;            // Handles quadrature encoder counting

// ============================================================================
// Forward Declarations (keep structure tidy)
// ============================================================================
// Output helpers (ALL Serial printing is centralized here)
void printPrompt();
void printStartRun(int duty);
void printRunningUpdate(int duty, double rpm, double revolutions);
void printTotals(double avgRpm, double totalRevs);
void printOutOfBounds(int minDuty, int maxDuty);
void printParseError(const String& line);
void printNotRunningMsg();
void printDutyChangeNotAllowed();
void printEncoderInitError();

// Motor control helpers
void motorStop();
void motorForwardDuty(uint8_t dutyPercent);
void motorBrake();

// Non-blocking Serial input
String readLineNonBlocking();

// ============================================================================
// setup()
// Purpose: Initialize Serial, pins, PWM, and encoder. Show initial prompt.
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

  // Show usage instructions to the student
  printPrompt();
}

// ============================================================================
// loop()
// Purpose: Process user input, run the motor, and print periodic measurements.
// ============================================================================
void loop() {
  // Keep encoder state fresh each iteration
  encoder.update();

  // ------------------------- Persistent State Variables ----------------------
  static bool running = false;                // True after a valid duty starts the motor
  static unsigned long lastPrintMs = 0;       // Time of last periodic print (millis)
  static double rpmSum = 0.0;                 // Accumulator for computing average RPM
  static unsigned long rpmSamples = 0;        // Number of samples accumulated
  static long startSubsteps = 0;              // Encoder substeps at start of current run
  static int lastDuty = 0;                    // The duty used for this run

  // Timestamp-based RPM anchors
  static long prevSubsteps = 0;               // Substeps at last RPM sample
  static uint32_t lastSampleUs = 0;           // micros() at last RPM sample

  // NEW: Latch when revolutions limit is reached
  static bool limitReached = false;

  // NEW: Throttle a tiny visual cue while braking (one dot per second)
  static unsigned long lastBrakeDotMs = 0;

  // ------------------------- 1) Non-blocking User Input ---------------------
  String line = readLineNonBlocking();

  if (line.length() > 0) {
    line.trim();

    // --- A) Stop command: 'x' ---
    if (line.equalsIgnoreCase("x")) {
      if (running) {
        // Brake briefly, then stop with outputs disabled (safe, repeatable end state)
        motorBrake();
        delay(250);
        motorStop();

        // Compute run statistics
        const double avgRpm   = (rpmSamples > 0) ? (rpmSum / (double)rpmSamples) : 0.0;
        const double totalRev = (double)(encoder.position - startSubsteps) / SUBSTEPS_PER_REV;

        // Print totals in one place
        printTotals(avgRpm, totalRev);

        // Reset state so the next run starts clean
        running      = false;
        lastDuty     = 0;
        rpmSum       = 0.0;
        rpmSamples   = 0;
        limitReached = false;

        // Reset revolution baseline to current position (next run starts at 0 rev)
        startSubsteps = encoder.position;

        // Reset timestamp anchors to avoid stale deltas on next run
        prevSubsteps = startSubsteps;
        lastSampleUs = micros();

        // Prompt for the next command
        printPrompt();
      } else {
        // User asked to stop, but we are already stopped
        printNotRunningMsg();
      }
    }
    // --- B) Numeric duty input ---
    else {
      // IMPORTANT: Duty changes are NOT allowed mid-run.
      if (running) {
        // Running: only 'x' is valid; ignore any numeric input
        printDutyChangeNotAllowed();
      } else {
        // Not running: parse integer duty and potentially start a run
        bool isNumber = true;
        for (size_t i = 0; i < (size_t)line.length(); i++) {
          if (!isDigit(line[i])) { isNumber = false; break; }
        }

        if (isNumber) {
          int duty = line.toInt();

          if (duty >= MIN_DUTY && duty <= MAX_DUTY) {
            // Start a new run at the requested duty
            motorForwardDuty((uint8_t)duty);
            lastDuty = duty;
            running = true;

            // Initialize timers for periodic printing
            lastPrintMs = millis();

            // Set revolution baseline at the current position
            startSubsteps = encoder.position;

            // Initialize timestamp-based RPM anchors
            prevSubsteps = startSubsteps;
            lastSampleUs = micros();

            // Reset average accumulator for this fresh run
            rpmSum = 0.0;
            rpmSamples = 0;

            // Reset limit latch and dot timer
            limitReached = false;
            lastBrakeDotMs = millis();

            // Inform the user that the run has started
            printStartRun(duty);
          } else {
            // Duty is numeric but outside allowed range
            printOutOfBounds(MIN_DUTY, MAX_DUTY);
          }
        } else {
          // Not 'x' and not an integer → parse error with guidance
          printParseError(line);
        }
      }
    }
  }

  // ------------------- 2) Periodic Status While Running ---------------------
  if (running) {
    // Suppress periodic status outputs while actively braking due to limit.
    // Show a tiny visual heartbeat: one dot per second.
    if (limitReached) {
      unsigned long nowMs = millis();
      if (nowMs - lastBrakeDotMs >= 1000UL) {
        Serial.print(".");
        lastBrakeDotMs = nowMs;
      }
      return; // still accept 'x' at the top on next loop iteration
    }

    const unsigned long nowMs = millis();

    // Print at a controlled cadence (e.g., 250 ms) for readability and consistency
    if (nowMs - lastPrintMs >= RPM_SAMPLE_MS) {
      lastPrintMs = nowMs;

      // Snapshot encoder position and current time
      long currSubsteps = encoder.position;  // cumulative encoder substeps
      uint32_t nowUs    = micros();          // current time in microseconds

      // Compute elapsed time in seconds for this sampling interval
      double dt_s = (double)(nowUs - lastSampleUs) / 1e6;

      // Guard: if dt is very small or negative (wrap/edge case), resync and skip output
      if (dt_s <= 0.0) {
        lastSampleUs = nowUs;
        prevSubsteps = currSubsteps;
        return;
      }

      // Change in substeps since last sample
      long deltaSubsteps = currSubsteps - prevSubsteps;

      // Revolutions relative to the start of THIS run (students see clean 0.000000 at start)
      double revolutions = (double)(currSubsteps - startSubsteps) / SUBSTEPS_PER_REV;

      // Convert delta substeps to rev/s, then to RPM
      double rev_per_s = ((double)deltaSubsteps / SUBSTEPS_PER_REV) / dt_s;
      double RPM       = rev_per_s * 60.0;

      // Update running average
      rpmSum     += RPM;
      rpmSamples += 1;

      // Print the neat single-line status
      printRunningUpdate(lastDuty, RPM, revolutions);

      // NEW: enforce max revolutions limit
      if (MAX_REVOLUTIONS > 0.0 && revolutions >= MAX_REVOLUTIONS) {
        // Immediately apply electrical braking
        motorBrake();
        limitReached = true;

        // One-time prompt. Periodic prints are now suppressed until 'x'.
        Serial.println("Max revolutions reached. Braking applied.");
        Serial.println("Type 'x' then Enter to finalize stop and view totals.");
        Serial.print("> ");

        // Initialize dot timer so the first dot appears after ~1 second
        lastBrakeDotMs = millis();
      }

      // Update anchors for next cycle
      prevSubsteps = currSubsteps;
      lastSampleUs = nowUs;
    }
  }
}

// ============================================================================
// Output Helpers
// Centralize ALL Serial prints for a tidy control flow in loop().
// ============================================================================

/*
  Function: printPrompt
  Purpose : Show instructions and the input prompt to the user.
*/
void printPrompt() {
  Serial.println();
  Serial.print("Enter duty cycle (");
  Serial.print(MIN_DUTY);
  Serial.print(" to ");
  Serial.print(MAX_DUTY);
  Serial.println(") then press Enter.");
  Serial.println("While running: only 'x' + Enter is accepted to brake & stop.");
  Serial.print("> ");
  Serial.flush();
}

/*
  Function: printStartRun
  Purpose : Confirm that a new run has started at the given duty.
*/
void printStartRun(int duty) {
  Serial.print("Running at ");
  Serial.print(duty);
  Serial.println("% duty. Type 'x' + Enter to brake and stop.");
}

/*
  Function: printRunningUpdate
  Purpose : Print the periodic line showing duty, RPM, and revolutions.
            Revolutions are relative to the start of the current run.
*/
void printRunningUpdate(int duty, double rpm, double revolutions) {
  Serial.print("Duty: ");
  Serial.print(duty);
  Serial.print("% | RPM: ");
  Serial.print(rpm, 2);
  Serial.print(" | Revolutions: ");
  Serial.println(revolutions, 6);
}

/*
  Function: printTotals
  Purpose : Report the average RPM and total revolutions when a run stops.
*/
void printTotals(double avgRpm, double totalRevs) {
  Serial.print("\nAverage RPM: ");
  Serial.print(avgRpm, 3);
  Serial.print(" | Total Revolutions: ");
  Serial.println(totalRevs, 6);
}

/*
  Function: printOutOfBounds
  Purpose : Inform the user that the duty value is outside the allowed range.
*/
void printOutOfBounds(int minDuty, int maxDuty) {
  Serial.print("Invalid input: duty must be between ");
  Serial.print(minDuty);
  Serial.print(" and ");
  Serial.print(maxDuty);
  Serial.println(" inclusive.");
  Serial.print("> ");
}

/*
  Function: printParseError
  Purpose : Explain that the input was not a valid integer or 'x'.
*/
void printParseError(const String& line) {
  Serial.print("Invalid input: '");
  Serial.print(line);
  Serial.println("'. Enter an integer duty in range, or 'x' to stop.");
  Serial.print("> ");
}

/*
  Function: printNotRunningMsg
  Purpose : Clarify that 'x' was entered while the motor was not running.
*/
void printNotRunningMsg() {
  Serial.println("Motor is not running. Enter a duty (then Enter) to start.");
  Serial.print("> ");
}

/*
  Function: printDutyChangeNotAllowed
  Purpose : Enforce the rule: cannot change duty while running.
*/
void printDutyChangeNotAllowed() {
  Serial.println("Duty cannot be changed while the motor is running.");
  Serial.println("Type 'x' + Enter to stop, then enter a new duty.");
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
  Function: motorForwardDuty
  Purpose : Drive the motor forward at a given duty. Constrains duty to [0, 100].
*/
void motorForwardDuty(uint8_t dutyPercent) {
  dutyPercent = constrain(dutyPercent, 0, 100);

  // Enable H-bridge outputs
  digitalWrite(STBYPin, HIGH);

  // Forward direction on TB6612FNG: AIN1 HIGH, AIN2 LOW (swap if wiring reversed)
  digitalWrite(AIN1Pin, HIGH);
  digitalWrite(AIN2Pin, LOW);

  // Apply PWM at the configured frequency
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, (float)dutyPercent);
}

/*
  Function: motorBrake
  Purpose : Short-brake (electrical braking) with both inputs HIGH at 0% duty.
            This increases damping and slows the motor more quickly than coasting.
*/
void motorBrake() {
  digitalWrite(STBYPin, HIGH);
  digitalWrite(AIN1Pin, HIGH);
  digitalWrite(AIN2Pin, HIGH);
  motorPWM->setPWM(pwmPin, PWM_FREQ_HZ, 0.0f);
}

// ============================================================================
// Non-blocking Serial Line Reader
// Reads a full line without blocking loop execution.
// Returns an empty String if a full line hasn't arrived yet.
// Handles CRLF by ignoring '\r' and terminating on '\n'.
// ============================================================================
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
