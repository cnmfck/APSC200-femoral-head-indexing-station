# Controller configuration

The final program is preserved unchanged. Values below are read from `anti_stall_encoder_sign_flip.ino`.

## Station mapping

`motor target = output command × 7.5 / 4.0`

| Station | Task's nominal absolute output angle | Saved absolute output command | Saved motor target |
| --- | --- | --- | --- |
| Picking | 0° | 0° | 0° |
| Laser | +108° | +116° | +217.5° |
| Mapping | −90° | −99° | −185.625° |

The saved relative commanded moves are +116°, −215°, and +99°. These are command-space values for the original mechanism. They do not establish the physical output trajectory independently of belt behavior.

## Tuning and timing

| Parameter | Saved value |
| --- | --- |
| Driver / driven diameters | 4.0 / 7.5 cm |
| Motor-to-output angle factor | 1.875 |
| Motor direction convention | +1.0 |
| Encoder sign | −1.0 |
| Counts per gearbox-output revolution constant | 374.22 |
| Library conversion | `374.22 × 4 × 64` substeps per revolution |
| PWM frequency | 1500 Hz |
| Maximum absolute duty | 45% |
| Normal minimum nonzero duty | 35% |
| Near-target duty floor | 32%, within 2° estimated output error |
| Requested stall kick | 65%, clamped to 45% by the final limit |
| Progress-check interval | 300 ms |
| Stall criterion | Less than 2° motor progress with output error greater than 2° |
| PID gains Kp / Ki / Kd | 0.65 / 0.010 / 0.03 |
| PID wind-up limits | −10 to +10 |
| Estimated output tolerance | 1° (equivalent to 1.875° motor error) |
| Brake delay | 250 ms |
| Dwell at laser / mapping | 13,500 / 9,000 ms |
| Motion serial interval | 250 ms |

The 65% kick request occurs on a progress-check iteration and is capped at 45%; the implementation does not provide a timed 65% boost. Some historical comments mention 100% availability, but the saved limit is 45%.

## Sequence and stopping

The state machine proceeds through `MOVE_TO_LASER`, `DWELL_AT_LASER`, `MOVE_TO_MAPPING`, `DWELL_AT_MAPPING`, `MOVE_TO_PICKING`, and `CYCLE_COMPLETE`. Serial `x` enters `ABORTED` after the brake-and-disable routine.

`enterDwell()` brakes for 250 ms, disables motor output, then starts the dwell timer. PID is stopped during dwell; there is no active position hold. Serial `r` resets the encoder zero at the current physical position, so manual re-homing is required first.
