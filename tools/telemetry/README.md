# TiltGuard telemetry capture

Captures and plots the firmware's RTT channel-1 CSV stream: 100 Hz
snapshots of the balance controller while it is armed (angle, rate,
setpoint, the law's terms, the driver command per wheel, encoder edges,
mode). Channel 0's human-readable text is separate - use RTT Viewer for
that; it's also where you type the console commands (see below). This
tool only reads channel 1.

## Setup

```
pip install -r requirements.txt
```

Needs a J-Link connection to the FPB-RA6T3's on-board debugger (J8) and
the firmware already running (past `telemetry_init()` in `hal_entry.c`).

## Capture

```
python capture.py capture.csv
```

Runs until Ctrl+C. Use `--duration 30` to stop automatically after 30
seconds, or `--device <name>` if you're not using the default
`R7FA6T3BB`. Rows only arrive while armed; the firmware sends a header
line at every arm, so one file can hold several runs (the script prints
`run N: armed` as each one starts).

## Plot

```
python plot.py capture.csv            # one figure per armed run
python plot.py capture.csv --run 2    # only run 2
python plot.py capture.csv --save out.png   # out_run1.png, out_run2.png, ...
```

Five panels per run: angle and setpoint (with the +/-2 deg logic-check band
and the +/-25 deg fault limit); gyro rate; the law's terms `bal`, `spd`,
`turn` in stock 8-bit units; the driver command per wheel as signed duty
(FWD +, REV -) with red marks where a wheel's direction doesn't match
`sign(ang - sp)` outside the 2 deg band; and encoder edges per 40 ms. The
title says DRY-RUN or ACTIVE and counts the mismatches. The firmware's own
`sign_err` also skips samples where D/turn outweigh half the P term (the
plot can't - it doesn't know kp/kd), so a red mark during a fast swing can
be legitimate PD braking; `sign_err` is the authoritative count.

## CSV format

```
t_ms,ang,sp,gyro,bal,spd,turn,dirL,dutyL,dirR,dutyR,encL,encR,motL,motR,mode
```

| Column | Meaning |
|---|---|
| `t_ms` | 1 kHz tick count = milliseconds |
| `ang` | Filtered pitch angle, degrees; > 0 = tilted forward |
| `sp` | Balance setpoint, degrees (changes live with `sp`) |
| `gyro` | `gyro_x` pitch rate, deg/s |
| `bal` | Balance PD term `kp*(ang-sp) + kd*gyro`, stock 8-bit units (+/-255 = full), before clamping |
| `spd` | Speed PI term, same units (0 while the speed loop is off) |
| `turn` | Turn (yaw-rate) term, same units. `pwm_L = clamp(bal - spd - turn)`, `pwm_R = clamp(bal - spd + turn)` |
| `dirL` / `dirR` | Driver command direction, decoded from the command itself (AIN1/BIN1 level and duty): `FWD`, `REV`, `STOP` |
| `dutyL` / `dutyR` | Duty written to the driver, permille 0..1000 (in DRY-RUN: the duty that would have been written) |
| `encL` / `encR` | Encoder edges in the last complete 40 ms window, unsigned (single-channel encoders) |
| `motL` / `motR` | Did the wheel turn in that 40 ms window: `ok` (driven, edges), `NONE` (driven >= 30% the whole window, 0 edges), `low` (driven below 30%, 0 edges - not judged), `idle` (not driven), `UNEXP` (edges while not driven). ~0.5 s of `NONE` latches FAULT `0x4` |
| `mode` | `ACTIVE` (motors driven) or `DRY-RUN` (STBY held low, PWM 0) |

Every row comes from one consistent snapshot of the most recent balance
step (`balance_get_status()`). The same fields go to channel 0 at 10 Hz as
the readable armed line:

```
ang=+3.2 sp=+0.0 gyro=-12.1 | bal=+176 spd=+0 turn=+0 | L=FWD 690 R=FWD 690 | encL=45 encR=44 | mot L=ok R=ok | ACTIVE
```

This must stay in sync with `CSV_HEADER` / `report_armed_csv()` in
`firmware/src/hal_entry.c` - change `capture.py`, `plot.py` and this table
together.

## Button only (no console needed)

As on the stock ELEGOO robot: set it on the floor (resting on its stand is
fine, anything within +/-22 deg) and press S1 in READY - it stands up and
balances. S1 while armed disarms; after a fall (FAULT) one S1 resets and
stands it up about 0.5 s later. The speed loop is held off for 0.5 s after each arm and
fades in over 1 s, so the stand-up kick does not wind it up.

## Console (RTT Viewer, channel 0)

Optional - everything above works without it.

Type a command into RTT Viewer's input line and press Enter. The space
before the value is optional (`skp0` = `skp 0`). Gains are in the stock
ELEGOO 8-bit PWM units (see `firmware/src/control/balance.h`). Each change
is applied atomically between two control steps and the full parameter set
is echoed back.

| Command | Meaning | Boot default | Accepted |
|---|---|---|---|
| `kp <v>` | balance P | 55 | 0..200 |
| `kd <v>` | balance D | 0.75 | 0..20 |
| `sp <deg>` | balance setpoint | 0 (stock) | -20..20 |
| `skp <v>` | speed P | 10 (stock; 0 with `BALANCE_SPEED_LOOP_OFF`) | 0..50 |
| `ski <v>` | speed I | 0.26 (stock; 0 with `BALANCE_SPEED_LOOP_OFF`) | 0..5 |
| `tkd <v>` | turn (yaw-rate) D | 0.5 | 0..10 |
| `dry <0\|1>` | dry-run off/on: armed, the law runs and logs, motors stay off | 0 (off; `BALANCE_DRY_RUN_AT_BOOT`) | refused while armed |
| `jog <pct>` | both wheels at +pct duty for 1 s, then stop; prints per-wheel encoder edges with a `moved` / `DID NOT MOVE` verdict and a pin/timer readback. Direction still needs an eye: they must roll FORWARD | | 1..20, READY only (works in dry-run) |
| `show` | print all parameters | | |
| `help` | list commands | | |

While armed, channel 0 also prints `check: samples=<n> sign_err=<n>` once a
second and `logic check: samples=<n> sign_err=<n>` at disarm. With the
speed loop on (`skp`/`ski` non-zero) the check is skipped and says so.

RTT Viewer's input works best set to send on Enter (Input -> Sending
behaviour). Values are not saved; a reset brings back the defaults in
`balance.h`.

## Alternative: J-Link RTT Logger instead of this script

SEGGER's J-Link software ships `JLinkRTTLogger` (same install as the
J-Link driver), which can log a single RTT channel to a file without
Python at all:

```
JLinkRTTLogger -Device R7FA6T3BB -if SWD -speed 4000 -RTTChannel 1 capture.csv
```

Flag names confirmed against the version installed with this project's
toolchain; check `JLinkRTTLogger` with no arguments (it prompts
interactively) or its own docs if your installed version differs. The
firmware sends the header line itself at every arm, so the log works with
`plot.py` as long as logging started before arming (rows before the first
header are ignored).
