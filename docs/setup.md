# Setup and wiring

## Development environment

Use the original Raspberry Pi Pico Arduino setup where available. The supplied course header records these dependencies:

| Library | Recorded version |
| --- | --- |
| RP2040_PWM, Khoi Hoang | 1.7.0 |
| PicoEncoder, Paulo Marques | 1.1.1 |
| ArduPID, PowerBroker2 | 0.2.1 |

The exact Arduino board-core version was not recorded in the supplied archive. Use a Pico/RP2040 core compatible with these libraries and verify the build. Libraries are external dependencies; no Arduino toolchain was available for compilation during packaging.

## Pins

| Connection | Pico GPIO number |
| --- | --- |
| TB6612FNG AIN1 | GP12 |
| TB6612FNG AIN2 | GP13 |
| TB6612FNG STBY | GP14 |
| TB6612FNG PWMA | GP15 |
| Encoder A | GP16 |
| Encoder B | GP17 |

GPIO numbers are not physical header-pin numbers. The encoder initializer uses `encoder.begin(16)` with consecutive A/B pins. Follow the original provided power board and motor-supply arrangement, with a shared ground as recorded in the course example. Power ratings and a full wiring schematic are not supplied.

## Load the final program

1. Install the compatible Pico board support and the three listed libraries in Arduino IDE.
2. Open `firmware/anti_stall_encoder_sign_flip/anti_stall_encoder_sign_flip.ino`.
3. Select the Raspberry Pi Pico board and its port, then use Verify to compile.
4. With actuator power off, position the lever at the picking station. Check belt alignment, clearance, and the original direction/encoder conventions before energizing the mechanism.
5. Upload the sketch. Set Serial Monitor to **115200 baud** and a line ending containing **newline**.
6. The sketch waits for a serial connection and then starts the first cycle automatically. Opening Serial Monitor can therefore start motion.
7. Send `x` followed by Enter to abort. After motion has stopped, remove actuator power as needed to re-position the mechanism, place it at picking, and send `r` followed by Enter for another cycle when ready.

`r` is a reset-and-start command, not an automatic homing command. `x` is a software serial abort; it depends on the controller and serial connection. There is no dedicated emergency-stop input in this sketch.

## Other sketches

Each folder is a standalone Arduino sketch. Open only one at a time; do not combine them. `encoder_debug` reads encoder movement without commanding motor motion, while `debugcode1` includes motor-control diagnostics. Course examples and archived controllers have their own behavior and may use different tuning or startup behavior.
