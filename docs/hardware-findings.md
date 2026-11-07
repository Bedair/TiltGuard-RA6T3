# TiltGuard hardware findings

Referenced from `CLAUDE.md`. Facts verified against ELEGOO's schematic
(`BalanceCarRobot-SCH-V06`), the stock Tumbller firmware, and this board's
own measurements - not assumptions. See `docs/wiring.md` for the pin map
and as-built wiring changes.

## OPEN ISSUE: battery-sense ADC reads ~9-10% high (deferred to Phase 4)

`v_bat` reads ~9-10% high and decays from ~13 V down to steady-state over
the first few seconds after reset (the decay itself is just the low-pass
filter's own settling tail following an unsettled first ADC conversion -
see the "Battery sense" section below; that part is understood). The
**steady-state 9-10% high offset is still open**.

Candidate explanation: with only a single GND jumper connecting the robot
board's ground to the FPB's ground, the two grounds were measured to
differ by **~0.15 V** at the current draw present during testing. Since
the battery sense is a single-ended measurement referenced to the FPB's
own ground, a ground offset there would show up directly as measurement
error, roughly consistent with the size of the discrepancy.

**This has not been confirmed.** The ADC input has not yet been tested
directly against the FPB's own GND and a known voltage derived from the
FPB's 3.3 V rail, with the robot board's divider/wiring taken out of the
loop - that direct test is what would isolate a pure ground-offset error
from other candidates (resistor divider tolerance, a wiring fault on the
sense line itself, etc.). Extra GND wires were added between the two
boards as a mitigation, but the battery reading was not re-verified
against that change.

**Next step (Phase 4):** test AN002 against FPB GND/3.3 V directly,
without the robot board in the loop, to confirm or rule out the
ground-offset explanation before trusting `v_bat` for the safety monitor's
low-battery cutoff.

Why it matters: digital signals (encoders, Echo) have enough logic-level
margin that the same offset, if present, wouldn't visibly affect them -
but that's margin, not immunity, and shouldn't be relied on as grounding
degrades further (e.g. under higher motor current once Phase 2's motors
are live).

## MPU-6050 (IMU)

- I2C/I3C-legacy address `0x68`. `WHO_AM_I` (register `0x75`) reads back
  `0x68`.
- Mounted with the same orientation as the ELEGOO Tumbller's reference
  board: accel angle = `atan2(accel_y, accel_z)`, pitch rate = `gyro_x`
  (confirmed by reading ELEGOO's own `KalmanFilter.cpp`, not assumed).

## Encoders

- Single-channel (A only). `M2A` = left, `M1A` = right. Direction comes
  from the motor command (Phase 2), not from the encoder signal itself.
- `COUNTS_PER_WHEEL_REV = 660`: 22 edges per motor revolution (11 pulses,
  both edges counted) x 30:1 gearing, matching ELEGOO's stock firmware
  figure.
- Confirmed on hardware (2026-11): ISR entry counts matched processed
  counts exactly on both wheels - no missed or double-counted edges found
  during testing.
- A hand test of 10 slow turns gave 694-746 counts/turn, which *overcounts*
  relative to 660 - expected, since hand-turning isn't perfectly
  one-directional on a single-channel encoder, and any back-and-forth
  jitter adds counts. This test does not confirm or refute 660; it only
  confirms the encoders and ISR path work.
- **To be verified in Phase 2**: drive the robot 1 m under motor control
  (smooth, one-directional motion) and check for ~3140 counts, which is
  the real test of the 660 figure.

## Ultrasonic (HC-SR04)

- Echo moved from the originally planned P407 (Pmod2-8) to P410 (Arduino
  D0) - P407 has no IRQ capability on RA6T3. See `docs/wiring.md` for the
  full reasoning.
- Confirmed on hardware (2026-11): stable 22.00 +/- 0.02 cm reading at a
  fixed distance, tracked a hand down to ~3 cm.

## Battery sense

- 10 k / 1.5 k divider: `V_bat = V_pin * 11.5 / 1.5`. Max ~1.10 V at the
  ADC pin at a full 2S pack (7.0-8.4 V nominal range).
- The ADC's first conversion after boot is unsettled (observed ~1.72 V on
  a reading that should have been close to the steady-state value) - the
  firmware now discards the first 4 scans after `R_ADC_Open()` before
  seeding the low-pass filter, rather than trusting the first sample.

## Battery and TB6612FNG / 74LVC2G14 (unchanged from original schematic review)

- Battery is 2S 18650 (7.0-8.4 V) on net `VM`. The robot board has no 5 V
  regulator - 5 V came from the Nano, now from the FPB's 5 V (bench,
  battery switch OFF during Phase 1) or a buck converter (Phase 2).
- The TB6612FNG and 74LVC2G14 run at 5 V; the RA6T3's 3.3 V outputs are
  below their VIH, so motor control lines must go through level shifters
  (Phase 2).
- RA6T3 5 V-tolerant pins: P100, P101, P205, P206, P400, P401, P407-P411.
  All other pins are 3.3 V max.
