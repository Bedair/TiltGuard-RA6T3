# TiltGuard status

## Milestones

### 2026-11-15: first successful floor balance

**What was seen** (from a 34 s hand-held video; no RTT log was captured, so no
angles, oscillation, saturation or fault data exist for this run):
- Released on the wood floor, not held. It stayed upright, unsupported, for the
  whole 34 s clip, with no fall and no visible FAULT.
- Poked about 8 times. It leaned visibly each time and recovered, including the
  largest lean (t ≈ 21–25 s), which showed the wheels and side plate from above.
- It stayed roughly in the same area. Drift couldn't be measured (handheld camera).
- Hardware: FPB-RA6T3 + ElectroCookie terminal shield mounted on top of the
  robot; some loose wire loops over the top.
- **Start-up differed from stock:** from rest on its support, it did not stand
  up. It was held near vertical and then S1 armed it. See "Start from rest" below.

**Build that balanced.** `git status` was clean on `main` at
`46ac052` ("Balance logic", 2026-11-15 14:06 +0100) when this was recorded,
and nothing had changed since that commit. That build had no build ID, so it
can't be proven the flashed image came from exactly this tree; from now on the
boot banner prints `build id: <hash>[-dirty] <date> <time>` (see below).

Boot defaults in `46ac052` (read from the tree with `git show`, not from memory):

| Item | Value | Where |
|---|---|---|
| kp | 55 (8-bit PWM per deg) | `BALANCE_KP_DEFAULT` |
| kd | 0.75 (per deg/s) | `BALANCE_KD_DEFAULT` |
| sp (setpoint) | 0.0 deg | `BALANCE_SETPOINT_DEG_DEFAULT` |
| skp | 10 | `BALANCE_SPEED_KP_DEFAULT` |
| ski | 0.26 | `BALANCE_SPEED_KI_DEFAULT` |
| tkd | 0.5 | `BALANCE_TURN_KD_DEFAULT` |
| Speed loop | **ON** (`BALANCE_SPEED_LOOP_OFF` commented out) | `balance.h` |
| Speed-loop soft start | held at 0 for 0.5 s after arm, then 1 s linear ramp | `BALANCE_SPEED_START_*` |
| Speed-loop anti-windup | integral frozen while both wheels saturate in its direction | `balance.c` |
| Dry-run at boot | off (motors live) | `BALANCE_DRY_RUN_AT_BOOT` |
| Angle filter | complementary, time constant 1 s at 1 kHz | `mpu6050.c` |
| Arming window | \|angle − sp\| < 22 deg | `BALANCE_ARM_WINDOW_DEG` |
| Tilt cut-out | \|angle\| > 25 deg → FAULT, **in every state** (also READY) | `SAFETY_TILT_LIMIT_DEG` |

These are the stock ELEGOO gains and units (see `firmware/src/control/balance.h`).
Any console changes made before the run aren't known; the recorder below now
stores the live values with every run.

**Start from rest: why it didn't stand up (facts, not a guess).**
- Stock (`Tumbller_Code_20191012`): `motion_mode` starts as `START`
  (`mode.h: } motion_mode = START;`). In `balanceCar()` the ±22° cut-out is
  skipped in START: `if (motion_mode != START && motion_mode != STOP &&
  (kalmanfilter_angle < balance_angle_min || balance_angle_max < kalmanfilter_angle))`
  with `balance_angle_min = -22; balance_angle_max = 22;`. In `loop()`,
  `case START: if (millis() - start_prev_time > 2000) { if (-22 <= angle <= 22)
  { car_speed_integeral = 0; motion_mode = STANDBY; } else { motion_mode = STOP; ... } }`.
  So stock drives the law at **any angle for the first 2 s** after power-on; from
  rest the angle term saturates (55 × 20° ≫ 255) and the wheels kick it upright.
  There is no arming window and no separate stand-up routine.
- Ours (`46ac052`): S1 armed only if |angle − sp| < **22°** (not ±10° as
  suspected), and the safety monitor latched a tilt FAULT whenever |angle| > **25°**,
  even in READY with the motors off. The resting angle on the support was
  −18.6° and −21.1° in the 2026-11-14 boot logs, i.e. right at the 22° edge, so
  depending on surface and how it settled S1 was either refused, or it armed and
  any excursion past 25° during the kick latched FAULT. Without a log it can't
  be told which; the refusal printed only on RTT, with no LED cue.
- Fix (this change): S1 arms from any |angle| ≤ 40°; for the first 2 s after arming
  the cut-out is ±45° (stock: none), then ±25° as before; the tilt check runs only
  while ACTIVE. A refused S1 prints `arm refused: angle=<x>, limit=+/-40 deg` and
  blinks LED1 at 10 Hz for 2 s. Every READY entry prints `READY: angle=<x>`, so the
  resting angle (on the support, and on its back if it can rest there) can be read
  off the log. **Not yet confirmed on hardware.**

**Added in the same change** (not yet confirmed on hardware): build ID in the boot
banner, the black-box recorder (`dump` / `tools/telemetry/dump.py`), the stand-up
start above, and the ElectroCookie wiring table in `docs/wiring.md`.

---

# Status report (read-only audit, 2026-11-08)

Generated 2026-11-08 by inspecting `configuration.xml`, `ra_gen/`, `src/`, and a real
build/link — not by reading `CLAUDE.md` or `docs/`. Where those docs differ from what's
actually in the tree, it's flagged below, not silently reconciled.

## 1. Build & repo state

- **Branch**: `main`, up to date with `origin/main`.
- **Last 5 commits**:
  ```
  0b18132 Phase 1: encoders, ultrasonic, battery ADC (battery open)
  40e1f5f Add MPU-6050 driver over I3C (legacy I2C), complementary filter
  867e5b7 Segger RTT configuration, Timer configuration
  f536d7c Initial project template
  ```
  Only 4 commits exist. Everything from the right-encoder pin moves, battery
  AN016 move, motors.c, safety.c, and the telemetry tooling is **uncommitted**.
- **Working tree**: all 29 changed/new files are currently **staged** (`git status`
  shows them under "Changes to be committed", not "Changes not staged"), no
  untracked files. Full list: `CLAUDE.md`, `docs/hardware-findings.md`,
  `docs/wiring.md`, 6 files under `firmware/.secure*`/`firmware/ra_cfg*`/
  `firmware/.settings` (FSP-generated metadata), 6 files under `firmware/ra_gen/`,
  `firmware/src/actuators/{motors.c,motors.h}` (new), `firmware/src/control/
  control_tick.{c,h}`, `firmware/src/hal_entry.c`, `firmware/src/imu/
  mpu6050.{c,h}`, `firmware/src/safety/{safety.c,safety.h}` (new),
  `firmware/src/sensors/{battery.c,battery.h,encoders.h}`,
  `firmware/src/telemetry/telemetry.{c,h}`, 4 new files under `tools/telemetry/`.
- **Build**: a real `make all` was run just now in `firmware/Debug` (GNU Make 4.1
  + the project's actual `arm-none-eabi-gcc 13.3 rel1`, same flags as
  `Debug/src/subdir.mk`: `-Wall -Wextra -Wconversion -Wshadow -Wfloat-equal
  -Waggregate-return -Wmissing-declarations -Wlogical-op -Wpointer-arith
  -Wuninitialized -Wunused`, `-std=c99`). Result: **links successfully, zero
  warnings, zero errors** (`EXIT: 0`). Image size: `text=20240 data=4 bss=3476`
  bytes (≈19.8 KB flash, ≈3.4 KB RAM out of 256 KB / 40 KB — plenty of headroom).
  All 12 `.c` files under `src/` were also compiled individually against the
  same flag set with the same result.

## 2. Clocks (`ra_gen/bsp_clock_cfg.h`)

| Clock | Source/config | Resulting frequency |
|---|---|---|
| XTAL | 20 MHz (populated but main osc not used as system source) | 20 MHz |
| HOCO | — | 20 MHz |
| PLL | Source HOCO, ÷2, ×20.0 | 200 MHz |
| System clock source | PLL | 200 MHz |
| ICLK (core) | ÷1 | 200 MHz |
| PCLKA | ÷2 | 100 MHz |
| PCLKB | ÷4 | 50 MHz |
| PCLKC | ÷4 | 50 MHz |
| PCLKD | ÷2 | 100 MHz |
| FCLK | ÷4 | 50 MHz |
| I3CCLK | Source PLL, ÷2 | 100 MHz |
| CLKOUT / UCLK / CANFDCLK | Disabled | — |

Peripherals actually depending on these: GPT0/1/2 run off PCLKD (100 MHz) per
the FSP GPT clock note; I3C0 runs off I3CCLK (100 MHz, independently divided
from PCLKD per RA6T3's dedicated I3C clock); ADC0 runs off PCLKC per the FSP
ADC clock note. ICU (external IRQ) has no clock dependency beyond the
digital filter (unused here — all three channels have `filter_enable=false`).

## 3. FSP stacks (`configuration.xml` / `ra_gen/hal_data.c` / `ra_gen/common_data.c`)

| Instance | Driver | Channel | Key settings | Interrupt(s) / priority | Callback |
|---|---|---|---|---|---|
| `g_timer_control_tick` | r_gpt (Timer) | 0 | `TIMER_MODE_PERIODIC`, period 50000 counts (`0xc350`), ÷2 source div → 1.000 ms (1 kHz), duty 50% (unused, no pin output) | Cycle-end, priority **2** | `control_tick_isr` |
| `g_pwm_motor_left` | r_gpt (Timer, PWM) | 1 | `TIMER_MODE_PWM`, period 5000 counts (`0x1388`), ÷1 source div → 50 µs (20 kHz), duty 0% at config (runtime-settable), GTIOCA out enabled, stop level low | None (`BSP_IRQ_DISABLED`) | none |
| `g_pwm_motor_right` | r_gpt (Timer, PWM) | 2 | Same as above | None | none |
| `g_i3c_imu` | r_i3c | 0 | `I3C_DEVICE_TYPE_MAIN_MASTER`, I2C-legacy mode, Standard Open-Drain 400000 Hz / 160 ns high period | `.ipl=5`, `.eei_ipl=5` | `mpu6050_i3c_callback` |
| `g_adc_battery` | r_adc | unit 0 | `ADC_MODE_SINGLE_SCAN`, `ADC_RESOLUTION_12_BIT`, scan mask = Channel 16 only, software-triggered | Scan-end, priority **6** | `battery_adc_callback` |
| `g_irq_encoder_left` | r_icu (External IRQ) | 7 | `EXTERNAL_IRQ_TRIG_BOTH_EDGE`, filter disabled | Priority **3** | `encoder_left_isr` |
| `g_irq_encoder_right` | r_icu (External IRQ) | 4 | `EXTERNAL_IRQ_TRIG_BOTH_EDGE`, filter disabled | Priority **3** | `encoder_right_isr` |
| `g_irq_echo` | r_icu (External IRQ) | 5 | `EXTERNAL_IRQ_TRIG_BOTH_EDGE`, filter disabled | Priority **4** | `ultrasonic_echo_isr` |
| `g_ioport` | r_ioport | — | Standard pin-table instance (`g_bsp_pin_cfg`) | — | — |

Vector table (`vector_data.h`), 11 entries, in priority order by vector index
(not by IPL — IPL is set independently per instance above):

| Vector | Source | IPL |
|---|---|---|
| 0 | GPT0 counter overflow | 2 |
| 1-6 | I3C0 (RCV_STATUS, RESPONSE, RX, TX, IBI, EEI) | 5 (all) |
| 7 | ICU IRQ7 (left encoder) | 3 |
| 8 | ICU IRQ4 (right encoder) | 3 |
| 9 | ICU IRQ5 (echo) | 4 |
| 10 | ADC0 scan end | 6 |

No duplicate vectors, no two active instances sharing one IRQn.

## 4. Actual pin map

Built from `ra_gen/pin_data.c` (electrical config) cross-referenced with
`configuration.xml`'s symbolic names (header label) and `ra_gen/common_data.c`/
`hal_data.c` (which peripheral instance, if any, owns the pin).

**Methodology note on the "5 V-tolerant" and "FPB header" columns**: this repo
has no RA6T3 hardware-manual datasheet file checked in, so "5 V-tolerant" below
is the fact set already on record in `CLAUDE.md` (`P100, P101, P205, P206,
P400, P401, P407-P411`), not re-derived from a fresh datasheet read in this
audit — flagged as such, not presented as newly verified. Tolerance only
matters electrically for pins receiving a 5 V signal (the three IRQ inputs and
the I3C pair); the five motor-control pins are level-shifted for **logic-level**
compatibility with the 5 V-powered TB6612/74LVC2G14, independent of whether the
FPB pin itself is rated tolerant. "FPB header" gives the Arduino/Pmod label from
`configuration.xml`; exact physical screw-terminal numbers (e.g. "J5-3") are
only included where a previous conversation turn explicitly stated them —
elsewhere this audit does not have independent header-pinout data to confirm a
specific J-number, and says so rather than guessing.

| MCU pin | FPB header | Function | Signal | Dir | Pull-up | 5V-tol. | vs. target |
|---|---|---|---|---|---|---|---|
| P409 | Arduino D5 | GPT1 Timer, PWM ch1, GTIOC1A | PWMA | Peripheral out | None | Yes (407-411 range) — irrelevant for an output pin | **OK** |
| P113 | Arduino D6 | GPT2 Timer, PWM ch2 | PWMB | Peripheral out | None | Not on the recorded list | **OK** |
| P008 | Arduino D7 | GPIO, output, low | AIN1 | Out | None | Not on the recorded list | **OK** |
| P110 | Arduino D12 (Pmod1 RX position) | GPIO, output, low | BIN1 | Out | None | Not on the recorded list | **OK** |
| P006 | Arduino D8 | GPIO, output, low | STBY | Out | None | Not on the recorded list | **OK** |
| P408 | Arduino D3 | r_icu channel 7, both edges | ENC_L | In | None | Yes | **OK** |
| P411 | Arduino D1 (Pmod2 MOSI position) | r_icu channel 4, both edges | ENC_R | In | None | Yes | **OK** |
| P410 | Arduino D0 (Pmod2 MISO position) | r_icu channel 5, both edges | ECHO | In | None | Yes | **OK** |
| P109 | Arduino D11 (Pmod1 TX position) | GPIO, output, low | TRIG | Out | None | Not on the recorded list (3.3 V direct per docs) | **OK** |
| P101 | Arduino SDA | I3C0, legacy I2C, 400 kHz | SDA | Peripheral, open-drain | None | Yes | **OK** |
| P100 | Arduino SCL | I3C0, legacy I2C, 400 kHz | SCL | Peripheral, open-drain | None | Yes | **OK** |
| P500 | Arduino D4 | ADC0 AN016 | VOL | Analog in | N/A | Not on the recorded list | **OK** |
| P207 | — | GPIO, output, low | LED1 | Out | None | — | **OK** |
| P206 | — | GPIO, output, low | LED2 | Out | None | — | **OK** |
| P000 (GPT0) | — | r_gpt Timer, channel 0, periodic | control tick | — | — | — | **OK** |

**All 15 target-map rows (13 signals + 2 LEDs, counting the tick separately) are
OK — exact match**, channel numbers and edge settings included.

**EXTRA** (configured in `pin_data.c`/`configuration.xml` but not part of the
target map):

| MCU pin | State found | Assessment |
|---|---|---|
| **P105** (Arduino D2) | `IOPORT_CFG_IRQ_ENABLE` still set | **Stale leftover** — this was the right encoder's intermediate pin (IRQ0) before the final move to P411. Flagged once already (previous status check) and still not cleared. Not a functional risk today (no `r_icu` stack claims IRQ0, nothing is wired to this pin anymore), but it's exactly the class of leftover that caused a real conflict risk with P402 earlier in the project. |
| **P002** (Arduino A2) | `IOPORT_CFG_ANALOG_ENABLE` | **Stale leftover**, cosmetic — the original VOL pin before the move to P500/AN016. Not scanned by the ADC's current channel mask; harmless but unused. |
| P400, P401 | GPIO output, high (I2C/I3C bus-mode select lines) | Required infrastructure for the I3C bus, not a leftover — intentionally outside the target map. |
| P304 | `IOPORT_CFG_IRQ_ENABLE` (S1 button, IRQ9) | Pre-existing board-template default, predates this project's changes, unused by any code. Harmless. |
| P108, P300 | Peripheral, DEBUG (SWDIO/SWDCLK) | Debug interface, required, unrelated to the application pin map. |
| P000, P001, P003, P004, P005, P013, P014, P015, P106, P107, P111, P200, P201, P208, P301, P403, P407 | Various (mostly `IOPORT_CFG_ANALOG_ENABLE` or plain input) | Unused board-template defaults. P111 and P402 were specifically checked (not just assumed) during the IRQ4-sharing analysis for the ENC_R move — **P111 confirmed clean** (plain input, no IRQ enabled); **P402 confirmed clean** (no entry at all in `pin_data.c` — its earlier stray IRQ4-enable has been fully removed, not just overlooked). |

**MISSING** / **DIFFERENT**: none found — every target signal is present and
matches.

## 5. Software modules in `src/`

| Module | Purpose | Public API | State | Hardware-verified |
|---|---|---|---|---|
| `telemetry/` | SEGGER RTT wrapper, two channels | `telemetry_init`, `telemetry_printf` (ch.0, readable), `telemetry_data_printf` (ch.1, CSV) | Implemented | Yes — RTT output has been read back in every hardware round so far |
| `control/control_tick` | 1 kHz GPT0 ISR, dispatches all `_on_tick()` hooks, tracks interval min/max and overrun count | `control_tick_init`, `control_tick_count`, `control_tick_get_interval_stats`, `control_tick_reset_interval_stats`, `control_tick_get_overrun_count`, `control_tick_isr` | Implemented | Yes — tick timing (cycles/1000 ticks ≈ 200 M) confirmed in Phase 1 |
| `imu/mpu6050` | MPU-6050 over I3C legacy-I2C; async burst read kicked from the tick; angle/filter math runs in the I3C read-complete callback, not the main loop | `mpu6050_init`, `mpu6050_on_tick`, `mpu6050_get_angle`, `mpu6050_get_sequence`, `mpu6050_get_missed_read_count`, `mpu6050_get_failed_read_count`, `mpu6050_get_last_read_cycles`, `mpu6050_get_last_control_step_cycles`, `mpu6050_i3c_callback` | Implemented | Yes — WHO_AM_I, 400 kHz bus, angle tracking all confirmed in Phase 1 |
| `sensors/encoders` | Both-edge pulse counting, 5 ms speed window | `encoders_init`, `encoders_on_tick`, `encoders_get_snapshot`, `encoder_left_isr`, `encoder_right_isr` | Implemented | Partial — ISR/counting path confirmed (entry counts == processed counts); `COUNTS_PER_WHEEL_REV=660` **not yet verified** by a driven-distance test |
| `sensors/ultrasonic` | HC-SR04 trigger/echo, 50 ms cycle, 30 ms timeout | `ultrasonic_init`, `ultrasonic_on_tick`, `ultrasonic_get_snapshot`, `ultrasonic_echo_isr` | Implemented | Yes — stable 22.00 ± 0.02 cm, tracked a hand to ~3 cm, on the current P410 pin |
| `sensors/battery` | ADC read, 100 ms cycle, exponential filter, warm-up discard | `battery_init`, `battery_on_tick`, `battery_get_snapshot`, `battery_adc_callback` | Implemented | Partial — 12-bit/divider conversion math verified correct on the *previous* pin (AN002/P002); **not yet re-tested on the current pin** (AN016/P500); open issue below unresolved either way |
| `actuators/motors` | GPT1/GPT2 PWM + AIN1/BIN1/STBY GPIO, ELEGOO-matching direction mapping | `motors_init`, `motors_set`, `motors_disable`, `motors_get_command` | Implemented | **No** — written and builds, but no PWM/direction multimeter test result has been reported back in this conversation |
| `safety/safety` | State machine `INIT→CALIBRATING→READY→ACTIVE→FAULT`, latched fault bitfield, calls `motors_disable()` on fault | `safety_init`, `safety_update`, `safety_request_active`, `safety_reset`, `safety_get_state`, `safety_get_fault_reasons`, `safety_get_warnings` | Implemented | **No** — the tilt-past-35°-latches-FAULT hardware test was planned but no result has been reported back |
| `control/` (balance law) | Cascaded angle→rate control (CLAUDE.md goal #1) | — | **Not started** — only `control_tick.c` exists in this directory; no control-law file | N/A |
| `hal_entry.c` | Startup sequencing, main loop, RTT reporting, a **temporary motor test pattern** | — | Implemented, but see note below | Partially (via the other modules) |

**Note on `hal_entry.c`**: once `safety` reaches `ACTIVE` (which it reaches
automatically and unconditionally ~0.5 s after boot — there's no real "start"
trigger yet), the main loop drives a **hardcoded test pattern** through
`motors_set()` (forward → stop → reverse → stop, ±300‰, 2 s/phase), explicitly
commented as temporary bring-up code, not a control law. This is a real,
active behavior difference from "the robot does nothing until Phase 3" — worth
knowing before connecting real motors.

## 6. Timing & real-time behaviour

**ISR inventory** (priority 0 = highest; nothing in this project uses 0 or 1):

| Priority | ISR | Does |
|---|---|---|
| 2 | `control_tick_isr` (GPT0, 1 kHz) | Updates tick count/interval stats/overrun count, then calls `mpu6050_on_tick()`, `encoders_on_tick()`, `ultrasonic_on_tick()`, `battery_on_tick()` in that fixed order. No FSP calls that block; starts async transfers only. |
| 3 | `encoder_left_isr`, `encoder_right_isr` (ICU) | Increment a raw counter. Nothing else. |
| 4 | `ultrasonic_echo_isr` (ICU) | Two-edge state machine: first call latches the rising timestamp, second computes pulse width → distance. Ignores edges when no measurement is in flight. |
| 5 | `mpu6050_i3c_callback` (I3C0, 6 vectors) | On write-complete, chains into the read; on read-complete, parses the 14-byte sample **and runs the accel-angle/complementary-filter math right there**, timed with `DWT->CYCCNT` (`mpu6050_get_last_control_step_cycles`). |
| 6 | `battery_adc_callback` (ADC0) | Reads the channel-16 result, updates the exponential filter (after a 4-scan warm-up discard). |

**Main loop** (`hal_entry.c`): polls `control_tick_count()`; on each new tick,
fetches snapshots from all sensor modules, calls `safety_update()`, runs the
temporary motor test pattern if `ACTIVE`, and conditionally emits telemetry at
four different rates (LED blink 1 Hz, CSV 200 Hz, sensor/safety text 10 Hz, IMU
text 2 Hz, tick-timing text 1 Hz). No blocking calls.

**IMU read timing, specifically**: `mpu6050_on_tick()` starts the write
(register pointer) + restart + 14-byte read every 1 kHz tick, unless the
previous transfer hasn't finished (counted as "missed"). **The control step
(angle/filter) is in the I3C read-complete callback, not deferred to the main
loop** — confirmed directly in `mpu6050.c`'s `mpu6050_tick_transfer_succeeded`,
called from `mpu6050_i3c_callback`'s `I3C_EVENT_READ_COMPLETE` case. This
matches what was asked for and reported before: a fresh sample is usable
within the I3C transfer's own duration (`mpu6050_get_last_read_cycles`,
typically a few hundred µs), not up to a full extra 1 kHz tick later.

## 7. Open issues & deviations

**Known, already-documented issues:**
- Battery ADC reads ~9-10% high, with a multi-second decay after reset (the
  decay itself is understood — the low-pass filter's own settling tail after
  an unsettled first conversion; the steady-state offset is not). A ground-
  offset between the robot and FPB (~0.15 V measured) is a candidate
  explanation, explicitly **not confirmed**. Deferred to Phase 4. Not
  re-tested since the move to AN016/P500.
- `COUNTS_PER_WHEEL_REV = 660` is ELEGOO's stock figure, not yet confirmed on
  this hardware by a driven-distance test (a hand-turn test was run but
  can't confirm or refute this value, by design — see `encoders.h`).

**New findings from this audit, not previously flagged as open:**
- **P105 still has a stray IRQ-enable bit** left over from the right
  encoder's retired intermediate pin (see Section 4). Not a functional risk
  currently, but unresolved.
- **Neither `motors.c` nor `safety.c` has a reported hardware test result**
  in this conversation — both are implemented and build clean, but "PWM
  frequency/duty on D5/D6", "direction pins toggle", and "tilt past ±35°
  latches FAULT" are all still open test items, not confirmed-working like
  the sensor modules are.
- **The main loop runs a hardcoded motor test pattern**, not a control law —
  worth being explicit about before connecting real motors, since it will
  start driving ~0.5 s after boot with no external trigger.
- **4 commits exist; most of the current tree is uncommitted** (staged, not
  committed) — if "the project" is judged by `git log` alone, it understates
  everything from the terminal-block wiring changes onward.

**Docs/CLAUDE.md vs. code — checked, no mismatches found** beyond what's
already listed above: `CLAUDE.md`'s pin map, clock facts, and `docs/wiring.md`/
`docs/hardware-findings.md`'s pin-history sections all match the current
`configuration.xml`/`ra_gen/` state as of this audit (they were updated
alongside the code in the same conversation that made these changes).

**TODO/FIXME in `src/`:**
- `firmware/src/safety/safety.h:23` — `TODO (later phase): this is a
  software-only cutoff with normal 1 kHz reaction latency. For a
  hardware-speed cutoff independent of this state machine, add a POEG...`
  (the only TODO/FIXME found in `src/`).

## 8. Next steps

**For Phase 2 (motors through the level shifters) to be considered actually
done, not just built:**
- Hardware test: PWM frequency/duty on D5/D6 (multimeter or LED), AIN1/BIN1
  toggling, with the terminal-block/shifter/buck-converter hardware
  connected — not yet reported.
- Replace or remove `hal_entry.c`'s hardcoded motor test pattern once real
  verification is done — it's bring-up scaffolding, explicitly not meant to
  stay.
- Re-test the battery reading on the new AN016/P500 pin (open issue, could
  turn out differently now that the ground path has changed with the
  terminal blocks).
- Clear the stray P105 IRQ-enable (cheap cleanup, not blocking).

**For the safety state machine to be considered verified:**
- The tilt-past-35°-latches-FAULT hardware test (motors disabled, latched
  until `safety_reset()`, visible in telemetry) — not yet reported.
- `SAFETY_FAULT_ENCODER` is reserved but unimplemented — no plausibility
  check exists yet; needs a model (e.g. comparing commanded direction/speed
  against encoder feedback) that doesn't exist until there's a real control
  law to compare against.
- No `safety_reset()` caller exists anywhere in `src/` yet — once latched,
  the only way to clear `FAULT` today is a full reset/reflash.

**For Phase 3 (balancing) to start:**
- No `control` module exists yet beyond `control_tick.c` (the tick
  infrastructure, not a control law). The angle/rate cascaded controller
  from `CLAUDE.md`'s goal #1 has not been started.
- The IMU-callback restructuring (Section 6) is already in place and ready
  for a control law to be added at that same call site, which was the
  explicit point of doing it now rather than later.

**In the FSP configuration specifically, nothing further is needed for what's
already wired** — GPT1/GPT2/ADC/I3C/all three ICU channels are configured and
match the target pin map exactly (Section 4). No pending FSP changes are
blocking Phase 2 or Phase 3 from a *configuration* standpoint; what's missing
is hardware test confirmation and the control-law code itself.
