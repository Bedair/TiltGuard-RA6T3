#!/usr/bin/env python3
"""
Capture TiltGuard's RTT channel-1 CSV stream to a file.

RTT channel 0 carries the human-readable status lines and the console -
keep RTT Viewer open for that. This script only reads channel 1.

The firmware sends channel-1 rows only while armed (S1 in READY, live or
dry-run), at 100 Hz, and sends a header line at every arm - so one file
can hold several runs, each starting with its own header. plot.py splits
them again. If capture starts in the middle of a run, the header is
written first so the file always parses.

CSV columns (must match report_armed_csv() / CSV_HEADER in
firmware/src/hal_entry.c - update both together). Every row is one
consistent snapshot of the most recent balance step:
    t_ms         1 kHz tick count = milliseconds since the tick started
    ang          filtered pitch angle, degrees (> 0 = tilted forward)
    sp           balance setpoint, degrees
    gyro         gyro_x pitch rate, deg/s
    bal          balance PD term, stock 8-bit units (+/-255 = full), unclamped
    spd          speed PI term, same units (pwm = bal - spd -/+ turn)
    turn         turn (yaw) term, same units
    dirL, dirR   driver command direction: FWD, REV or STOP
    dutyL, dutyR duty actually written to the driver, permille (0..1000);
                 in DRY-RUN the duty that would have been written
    encL, encR   encoder edges in the last complete 40 ms window, unsigned
    motL, motR   did the wheel turn in that window: ok, NONE (driven >= 30%,
                 no edges), low, idle (not driven), UNEXP (turned undriven)
    mode         ACTIVE (motors driven) or DRY-RUN (motors held off)

Requires pylink-square (`pip install -r requirements.txt`) and a J-Link
connection to the FPB-RA6T3's on-board debugger.
"""
import argparse
import sys
import time

import pylink

DEFAULT_DEVICE = "R7FA6T3BB"
RTT_CHANNEL = 1
CSV_HEADER = "t_ms,ang,sp,gyro,bal,spd,turn,dirL,dutyL,dirR,dutyR,encL,encR,motL,motR,mode"


def wait_for_rtt(jlink, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            if jlink.rtt_get_num_up_buffers() > RTT_CHANNEL:
                return True
        except pylink.errors.JLinkRTTException:
            pass
        time.sleep(0.05)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("output", help="CSV file to write")
    parser.add_argument("--device", default=DEFAULT_DEVICE,
                        help="J-Link target device name (default: %(default)s)")
    parser.add_argument("--duration", type=float, default=None,
                        help="Stop after this many seconds (default: run until Ctrl+C)")
    args = parser.parse_args()

    jlink = pylink.JLink()
    jlink.open()
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(args.device)
    jlink.rtt_start()

    if not wait_for_rtt(jlink):
        print("Timed out waiting for the target to publish its RTT control block "
              "(is the firmware running and past telemetry_init()?)", file=sys.stderr)
        jlink.rtt_stop()
        jlink.close()
        sys.exit(1)

    print(f"Connected to {args.device}. Writing channel {RTT_CHANNEL} CSV to {args.output}")
    print("Rows arrive only while armed (S1). Press Ctrl+C to stop.")

    start_time = time.monotonic()
    partial_line = ""
    lines_written = 0
    runs = 0
    first_line = True

    with open(args.output, "w", newline="") as f:
        try:
            while True:
                if args.duration is not None and (time.monotonic() - start_time) > args.duration:
                    break

                data = jlink.rtt_read(RTT_CHANNEL, 1024)

                if not data:
                    time.sleep(0.005)
                    continue

                partial_line += bytes(data).decode("ascii", errors="replace")

                while "\n" in partial_line:
                    line, partial_line = partial_line.split("\n", 1)
                    line = line.strip("\r")

                    if not line:
                        continue

                    if first_line and line != CSV_HEADER:
                        f.write(CSV_HEADER + "\n")  # Joined in the middle of a run.
                    first_line = False

                    if line == CSV_HEADER:
                        runs += 1
                        print(f"run {runs}: armed")

                    f.write(line + "\n")
                    lines_written += 1
        except KeyboardInterrupt:
            print("\nStopped by user.")

    jlink.rtt_stop()
    jlink.close()
    print(f"Wrote {lines_written} lines ({runs} armed runs) to {args.output}")


if __name__ == "__main__":
    main()
