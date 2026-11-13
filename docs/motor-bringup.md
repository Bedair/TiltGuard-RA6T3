# Motor bring-up (Phase 2, first power-on)

First test of the motors through the terminal-block/shifter/buck-converter
architecture (see `docs/wiring.md`). Read this all the way through before
touching anything.

## Before you start

- **Wheels lifted off the ground**, robot secured on a stand so both wheels
  spin completely freely. Do not skip this.
- **Battery switch OFF** until step 2 below — don't power the buck converter
  until you're ready to check its output.
- Firmware: build with `MOTOR_BENCH_TEST` defined in `hal_entry.c` (it's
  commented out by default) for this first test — without it, arming (S1)
  doesn't drive the motors at all, and you won't see anything move.
- Know where S1 is on the FPB and have a multimeter ready.

## Step-by-step

1. Robot secured, wheels free-spinning, battery switch still OFF.
2. Battery switch ON. This feeds the buck converter's IN+ from `VM`
   (7-8.4 V) — it's the only thing powering the buck converter, so this
   step is required to check its output at all.
3. **Before** the buck converter's OUT+ is connected to the robot's 5 V
   rail or either shifter's HV side: measure OUT+ to OUT- with a
   multimeter. Confirm ~5.00 V. Adjust the buck converter's trimmer if
   it's off *before* connecting it to anything else.
4. Connect buck OUT+ to the GERUI `5V` terminal and both shifters' HV rail,
   per `docs/wiring.md`.
5. Re-measure the robot's 5 V rail (GERUI `5V` terminal) now that it's
   under load (shifters, motor driver). Should still read close to 5.0 V —
   if it's sagging noticeably, stop and check the wiring before proceeding.
6. Flash the firmware (FPB on USB power, as always — separate from the
   robot's battery-fed 5 V rail).
7. Watch RTT channel 0. Confirm:
   - `mpu6050`, `encoders`, `ultrasonic`, `battery`, `motors`, `button`,
     `control_tick` all initialize without a `..._init failed` message
     (any failure holds LED2 solid on and halts — if you see that, stop
     and diagnose before continuing).
   - `safety=` in the 10 Hz telemetry line progresses
     `INIT` → `CALIBRATING` → `READY` over about 0.5 s.
8. Confirm **LED2 is off** (READY).
9. **Press S1 once** (short press). Expect:
   - `safety=ACTIVE` in telemetry, LED2 starts blinking at 1 Hz.
   - RTT prints `bench test: left fwd`.
10. **Left wheel** spins at ~15% duty for 2 s; right wheel stays still.
    "Forward" here means whatever `motors_set(+150, 0)` commands —
    AIN1 low, matching the ELEGOO stock firmware's `carForward()`
    convention (see `motors.c`). This test doesn't independently know
    which physical rotation direction that corresponds to on your
    assembled chassis — watch the wheel and compare against what
    "forward" should look like on this robot. If it's backwards, see
    the note below.
11. RTT prints `bench test: left rev`; left wheel spins the *opposite*
    direction from step 10, right wheel still stationary, 2 s.
12. RTT prints `bench test: right fwd`; right wheel spins, left stationary, 2 s.
13. RTT prints `bench test: right rev`; right wheel spins opposite, left stationary, 2 s.
14. RTT prints `bench test: done, disarming`; both wheels stop, LED2 goes
    off, `safety=READY` again.

## Abort / emergency stop

- **Press S1 again at any point** while `ACTIVE` (including mid-bench-test)
  → immediate disarm: `motors_disable()` runs (STBY low, both duties 0),
  regardless of which phase is running.
- **Tilt the robot past ±35° at any point**, including during the bench
  test → immediate latched `FAULT`: `motors_disable()` runs, LED2 goes
  solid on. This is checked every tick throughout the bench test, not
  skipped — confirmed by `safety_update()` running unconditionally before
  the bench-test logic even looks at the safety state. Recovering needs a
  press of S1 *in FAULT* (→ `safety_reset()` → `INIT`, which re-runs the
  0.5 s `CALIBRATING` settle before `READY`), then a second press to
  re-arm.
- **Hardware-level stop that doesn't depend on firmware at all**: battery
  switch OFF kills the buck converter, and with it the robot's 5 V rail and
  both shifters, immediately. Use this if anything looks wrong and you
  don't trust the software path.

## If a wheel spins the wrong way

Swap that motor's two terminal-block wires at the GERUI end (reverses the
physical winding direction) rather than changing `motors.c`'s AIN1/BIN1
mapping — the mapping in software is deliberately matched to the ELEGOO
stock firmware's convention; fix the direction in wiring, not by diverging
from that reference.

## Battery sense re-test (P500/AN016)

`VOL` moved from P002/AN002 to P500/AN016 to free up J6 (see
`docs/wiring.md`). This has **not been re-tested since the move**, and
there's a known, unconfirmed ~9-10% high reading open issue from before the
move, deferred to Phase 4 (see `docs/hardware-findings.md`).

1. With everything wired as above and the battery on, read `v_bat` and
   `v_bat_raw` from the RTT channel-0 10 Hz line.
2. Separately, measure the real battery pack voltage directly at the GERUI
   `VIN` terminal with a multimeter.
3. Compare `v_bat` (firmware) against the multimeter reading.

**Expected, not yet confirmed on this pin**: if the ~9-10% high offset is
still present unchanged, `v_bat` will read roughly 1.09-1.10× the
multimeter value (e.g. multimeter 7.4 V → `v_bat` ~8.0-8.1 V). If the
offset turns out to have been specific to the old P002 wiring path rather
than a general ground-topology issue, the new P500 path might read
noticeably closer to the multimeter value — that's the open question this
test is for. Record both numbers side by side rather than just a pass/fail;
this feeds the Phase 4 investigation either way.
