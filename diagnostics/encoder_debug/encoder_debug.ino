/*
  Minimal encoder diagnostic for APSC 200 RP2040 setup.
  Same encoder pins as the provided example.
  No motor motion is commanded.

  Purpose:
    - Verify that encoder counts change when the shaft is turned by hand.
    - Verify the sign/direction of encoder counts.

  How to use:
    1) Upload this file.
    2) Open Serial Monitor at 115200 baud.
    3) Turn the motor/encoder shaft slowly by hand.
    4) Watch whether raw counts and angle change.
*/

#include "PicoEncoder.h"

#define encAPin 16
#define encBPin 17

const float  COUNTS_PER_REV         = 374.22f;
const double STEPS_PER_REV          = COUNTS_PER_REV * 4.0;
const double SUBSTEPS_PER_REV       = STEPS_PER_REV * 64.0;
static const double DEG_PER_SUBSTEP = 360.0 / SUBSTEPS_PER_REV;

PicoEncoder encoder;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  if (encoder.begin(encAPin) != 0) {
    Serial.println("ERROR: Encoder init failed.");
    while (true) { delay(1000); }
  }

  encoder.resetPosition();
  Serial.println();
  Serial.println("Encoder check started.");
  Serial.println("Turn the shaft by hand and watch raw counts + angle.");
  Serial.println("Type 'z' + Enter to reset position to zero.");
}

void loop() {
  encoder.update();

  if (Serial.available() > 0) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.equalsIgnoreCase("z")) {
      encoder.resetPosition();
      Serial.println("Encoder position reset to zero.");
    }
  }

  static long lastPos = 999999999;
  long pos = encoder.position;
  if (pos != lastPos) {
    lastPos = pos;
    double angleDeg = ((double)pos) * DEG_PER_SUBSTEP;
    Serial.print("raw enc: ");
    Serial.print(pos);
    Serial.print(" | angle: ");
    Serial.print(angleDeg, 4);
    Serial.println(" deg");
  }

  delay(10);
}
