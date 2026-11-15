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

**Next step (Phase 4):** test the battery-sense ADC input (now AN016/P500,
moved from AN002/P002 - see `docs/wiring.md`) against FPB GND/3.3 V
directly, without the robot board in the loop, to confirm or rule out the
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
- Right encoder's pin moved twice, both times away from **P205 (Pmod2-7)**:
  **Pmod 2 isn't broken out on either the ElectroCookie (FPB) or GERUI
  (robot) terminal-block boards** - only the Arduino-header-shaped pins
  are, so P205 became physically unreachable once the terminal blocks
  replaced the jumper wires, regardless of its IRQ capability. Final pin
  is **P411 (IRQ4)**, direct (5 V-tolerant, no shifter needed). See
  `docs/wiring.md`'s pin-history section for the full IRQ4-sharing
  reasoning (P411 ended up sharing its interrupt channel with P111 and
  P402).

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
- Moved from A2/P002 (J6-3) to D4/P500 (AN016) specifically to **free up
  J6 entirely** - nothing else used it, and dropping it simplified the
  terminal-block wiring. Checked five free Arduino-header candidates
  against the pin-function database first: only P500 has any ADC function
  in silicon.

## Battery and TB6612FNG / 74LVC2G14

- Battery is 2S 18650 (7.0-8.4 V) on net `VM`. The robot board has no 5 V
  regulator - 5 V came from the Nano originally; in Phase 1 it was
  bench-powered from the FPB's own 5 V output (battery switch OFF); now
  (Phase 2) it comes from a buck converter set to 5.0 V, fed from the real
  battery through the GERUI terminal board. See `docs/wiring.md`.
- The TB6612FNG and 74LVC2G14 run at 5 V; the RA6T3's 3.3 V outputs are
  below their VIH, so motor control lines (PWMA, PWMB, AIN1, BIN1, STBY)
  go through BSS138 level-shifter modules - not yet tested with real
  motors, just the GPIO/PWM lines bench-checked on the FPB side so far.
- RA6T3 5 V-tolerant pins: P100, P101, P205, P206, P400, P401, P407-P411.
  All other pins are 3.3 V max.

## FINDING (2026-11-13): motor lines float high during reset, so the motors spin

**Observed:** at the first motor test, pressing S2 (RESET) on the FPB made
both motors spin, with the battery on and the buck converter feeding the
shifters' HV side.

**Cause:** while RES# is held low, and from reset release until the firmware
configures the pins, the RA6T3 GPIOs are Hi-Z inputs. Each BSS138 shifter
channel has a 10 k pull-up on its LV side (to FPB 3V3) and on its HV side
(to buck 5 V). With the LV pin floating, the LV pull-up holds the BSS138 off,
and the HV pull-up drives the TB6612FNG input high. STBY, PWMA, PWMB, AIN1
and BIN1 all go high together: the driver is enabled at 100% duty in reverse,
on both motors. The same happens at every power-on of the FPB, during a
debugger-held reset and during flashing.

**Hardware fix:** a 1 kOhm pull-down from the LV side of STBY to GND, and on
PWMA and PWMB as well (defence in depth; STBY low on its own turns off all
TB6612 outputs). See `docs/wiring.md`.

Sizing notes (calculated, **not yet measured**):
- The pull-down has to beat the LV 10 k pull-up, plus the HV 10 k pull-up
  whose current flows through the BSS138 into the LV node once the FET
  turns on. With 1 k: LV ≈ 0.7 V, so HV ≈ 0.7 V at the TB6612 input. Its
  VIL max is 0.8 V, which leaves about 0.1 V of margin. That should work,
  but check it: **hold S2 and measure STBY at the HV side (robot D8)**. It
  must read below 0.8 V, ideally around 0.5 V or less.
- If the margin is too small: remove that channel's LV-side 10 k pull-up
  (→ ≈ 0.45 V with 1 k), or go down to 680 Ohm.
- Cost: when the MCU drives the line high, the pin now sources about
  3.3 mA into the pull-down (more at 680 Ohm). With low drive capacity the
  pin's VOH may sag, and the BSS138 must stay fully off (Vgs below its
  ~0.8 V minimum Vth). Setting the pins to middle drive in the FSP
  configurator (Pins → P006/P409/P113 → Drive Capacity = Middle) removes
  that doubt.
- A pull-down on the **HV** side would be wrong: it fights the HV 10 k
  pull-up all the time, so the line could never reach a valid high.

**Firmware mitigation** (shortens the window after reset is released; it
cannot help while RES# is held):
- `R_BSP_WarmStart(BSP_WARM_START_RESET)` in `src/hal_warmstart.c` drives
  P006, P008, P110, P113 and P409 low through `PCNTR1`. It is the first
  user code after reset, before clock init and the C runtime.
- `R_IOPORT_Open()` at `BSP_WARM_START_POST_C` then sets the configured
  state: STBY/AIN1/BIN1 output-low, PWMA/PWMB on the GPTs at 0% duty.
- `motors_init()` is the first call in `hal_entry()`, before `telemetry_init()`.

## FINDING (2026-11-13): console command crashes into Default_Handler - main stack budget

**Observed:** typing a console command (e.g. `skp 0`) in RTT Viewer crashed
the firmware into `Default_Handler`.

**Analysis** (GCC `-fcallgraph-info=su`, worst-case call chains; the fault
dump from `src/safety/fault.c` is still to confirm it on hardware):
- The main stack is 1 KB (`BSP_CFG_STACK_MAIN_BYTES`) and `MSPLIM` is set to
  its base, so an overflow raises UsageFault STKOF, which went to
  `Default_Handler` before `fault.c` existed.
- The main loop needed up to **936 B** on the console path: `hal_entry`'s own
  frame (424 B, every report's locals live at once), the console's wide
  printf (120 B) and SEGGER's printf chain (~280 B). That left 88 B, less
  than one FP-extended exception frame (104 B), so any interrupt at that
  point overflowed.
- Interrupt nesting on top: each preemption costs 104 B if the interrupted
  code has used the FPU (the main loop always has), otherwise 32 B. Measured
  per ISR:

  | Prio | ISR | FP | Handler stack |
  |---|---|---|---|
  | 2 | GPT0 tick | no | 140 B |
  | 3 | encoders IRQ7/IRQ4 | no | 16 B |
  | 4 | echo IRQ5 | yes | 20 B |
  | 5 | I3C (IMU callback + balance step) | yes | 280 B |
  | 6 | ADC (battery) | yes | 112 B |
  | 7 | button IRQ9 | no | 16 B |

  Full nesting is about 1064 B on its own. The everyday case of the tick
  preempting the I3C/balance callback is about 630 B.

**Fix, keeping the 1 KB stack:**
1. Main loop reduced to **504 B** worst case (incl. `main()`): each report
   in `hal_entry.c` is a separate non-inlined function; the console prints
   one field per call and only after parsing has returned; the 200 Hz CSV
   uses `telemetry_line_*` (no printf, one atomic RTT write per row);
   `SEGGER_RTT_PRINTF_BUFFER_SIZE` 64 -> 32.
2. All interrupts at **one priority level** (FSP configurator), so ISRs never
   nest: worst case = 504 + 104 (frame) + 280 (I3C/balance) = **~890 B, about
   130 B margin**. Cost: an ISR may wait for the one running ahead of it (at
   most the tick ISR with its 10 us trigger pulse, or the IMU callback with the
   balance step). That's about 3 mm of echo-timestamp error, and no lost encoder
   edges at these edge rates.

The 1 Hz tick line prints `stack_free=<bytes>/<size>` from a painted stack,
which is the measured confirmation on hardware.

**Confirmed on hardware (2026-11-14):** no crash on console commands;
`stack_free` stayed at 404/1024 while balancing and typing commands.

**Update (dry-run / logic-check build):** the balance step now also plans
the driver command and runs the logic check, so the I3C path (dispatcher +
`mpu6050_i3c_callback` + `balance_step`) grew from 280 B to **336 B** with the
motion check (the command record is static to keep it off the stack). Main
loop unchanged at 504 B. New worst case 504 + 104 + 336 = **~944 B, about
80 B margin** (measured: `stack_free=372/1024` on the dry-run build).
Raising the stack: FSP configurator → BSP tab → Properties → RA Common →
Main stack size (bytes), e.g. `0x1000` (4 KB; RAM use is ~6.7 KB of 40 KB
today). `fault.c` and the `stack_free` print follow `BSP_CFG_STACK_MAIN_BYTES`
automatically.
