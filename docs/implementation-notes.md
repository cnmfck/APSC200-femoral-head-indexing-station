# Implementation notes

These observations document the unchanged user-confirmed successful version.

## Position feedback

The encoder measures rotation before the rubber-band transmission. Lever position is estimated using the fixed diameter ratio. Changes in slip, stretch, effective belt contact diameter, or compliance can cause actual lever position to differ from the estimate. The saved +116°/−99° targets are retained exactly; their numeric differences from the task targets should not be interpreted as independently measured errors.

## Anti-stall implementation

The controller applies a duty floor when the PID request is nonzero. Every 300 ms it checks motor-angle progress; when progress is small and error is large, it requests a stronger command toward the target. The 65% request is immediately clamped to the saved 45% maximum and has no separately timed pulse duration. No maximum move timeout or persistent-stall fault is implemented.

## Dwell behavior

The motor is briefly braked and then disabled for each dwell. The software does not actively correct displacement during the dwell, even though a variable is named `holdingPosition`. It begins dwell after one in-tolerance motor-angle check rather than requiring a settling period within tolerance.

## Logging and serial commands

The `Duty` field prints raw PID output, not the final command after duty-floor/anti-stall adjustments. `finishCycle()` and `abortCycle()` update the encoder after braking but do not recompute `inputDeg` before their final printed angle; that value may therefore be from before braking.

Serial commands are processed after newline. The 250 ms brake delay blocks serial processing temporarily. `r` is accepted during operation and resets the assumed zero to the current physical position; only use it after stopping and manually returning to picking.

## Verification scope

The author's confirmation establishes historical prototype operation. Packaging did not introduce source changes, recompile against the Arduino libraries, or repeat physical tests. Code findings are static observations, and the report's measured performance is documented separately in `results.md`.
