# TiltGuard telemetry capture

Captures and plots the firmware's RTT channel-1 CSV stream (200 Hz:
filtered pitch angle, gyro rate, motor command, encoder speed, safety
state). Channel 0's human-readable status text (`telemetry_printf` in
`hal_entry.c`) is separate - use RTT Viewer or the J-Link GDB Server's
telnet port for that; this tool only reads channel 1.

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
`R7FA6T3BB`.

## Plot

```
python plot.py capture.csv
```

Shows four stacked plots over time (derived from the 1 kHz tick column):
filtered pitch angle (with the ±35° fault limit marked), gyro rate, motor
command + encoder speed together, and the safety state machine. Pass
`--save out.png` to write a file instead of opening an interactive window.

## CSV format

```
tick,imu_seq,angle_centideg,gyro_centidps,motor_L_permille,motor_R_permille,enc_L_counts_per_5ms,enc_R_counts_per_5ms,safety_state
```

| Column | Meaning |
|---|---|
| `tick` | 1 kHz control-tick count when this line was generated |
| `imu_seq` | Count of completed IMU reads (`mpu6050_get_sequence()`) |
| `angle_centideg` | Filtered pitch angle, degrees x100 (divide by 100) |
| `gyro_centidps` | `gyro_x` rate, degrees/second x100 (divide by 100) |
| `motor_L_permille` / `motor_R_permille` | Motor command, -1000..1000. **Placeholder 0 until `motors.c` exists** - see `firmware/src/hal_entry.c`'s CSV block |
| `enc_L_counts_per_5ms` / `enc_R_counts_per_5ms` | Raw encoder speed |
| `safety_state` | `0`=INIT `1`=CALIBRATING `2`=READY `3`=ACTIVE `4`=FAULT |

This must stay in sync with the `telemetry_data_printf(...)` call in
`firmware/src/hal_entry.c` - if that line's field order or count changes,
update `CSV_HEADER` in `capture.py` and the column names here together.

## Alternative: J-Link RTT Logger instead of this script

SEGGER's J-Link software ships `JLinkRTTLogger` (same install as the
J-Link driver), which can log a single RTT channel to a file without
Python at all:

```
JLinkRTTLogger -Device R7FA6T3BB -if SWD -speed 4000 -RTTChannel 1 capture.csv
```

Flag names confirmed against the version installed with this project's
toolchain; check `JLinkRTTLogger` with no arguments (it prompts
interactively) or its own docs if your installed version differs. Its
output won't have the CSV header line `capture.py` adds, and you'd still
use `plot.py` to visualize it (add the header line manually, or pass
`--save`/adjust `plot.py` if `pandas.read_csv` complains about missing
column names).
