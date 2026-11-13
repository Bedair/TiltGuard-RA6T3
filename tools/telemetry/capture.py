#!/usr/bin/env python3
"""
Capture TiltGuard's RTT channel-1 CSV stream to a file.

RTT channel 0 carries the human-readable status lines (telemetry_printf in
hal_entry.c) - open that separately with RTT Viewer or the J-Link GDB
Server's telnet port if you want to watch it live. This script only reads
channel 1, the compact 200 Hz CSV stream (telemetry_data_printf), written
by firmware/src/hal_entry.c's CSV_REPORT_TICKS block.

CSV columns (must match hal_entry.c - update both together if the firmware
line changes):
    tick                    1 kHz tick count at the time of this sample
    imu_seq                 mpu6050_get_sequence() - counts completed IMU reads
    angle_centideg          filtered pitch angle, degrees x100 (int)
    gyro_centidps           gyro_x rate, degrees/second x100 (int)
    motor_L_permille        left motor command, -1000..1000 (0 until motors.c lands)
    motor_R_permille        right motor command, -1000..1000 (0 until motors.c lands)
    enc_L_counts_per_5ms    raw left encoder speed
    enc_R_counts_per_5ms    raw right encoder speed
    safety_state            0=INIT 1=CALIBRATING 2=READY 3=ACTIVE 4=FAULT

Requires pylink-square (`pip install -r requirements.txt`) and a J-Link
connection to the FPB-RA6T3's on-board debugger.
"""
import argparse
import sys
import time

import pylink

DEFAULT_DEVICE = "R7FA6T3BB"
RTT_CHANNEL = 1
CSV_HEADER = (
    "tick,imu_seq,angle_centideg,gyro_centidps,motor_L_permille,"
    "motor_R_permille,enc_L_counts_per_5ms,enc_R_counts_per_5ms,safety_state"
)


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
    print("Press Ctrl+C to stop.")

    start_time = time.monotonic()
    partial_line = ""
    lines_written = 0

    with open(args.output, "w", newline="") as f:
        f.write(CSV_HEADER + "\n")

        try:
            while True:
                if args.duration is not None and (time.monotonic() - start_time) > args.duration:
                    break

                data = jlink.rtt_read(RTT_CHANNEL, 1024)

                if data:
                    partial_line += bytes(data).decode("ascii", errors="replace")

                    while "\n" in partial_line:
                        line, partial_line = partial_line.split("\n", 1)
                        line = line.strip("\r")

                        if line:
                            f.write(line + "\n")
                            lines_written += 1
                else:
                    time.sleep(0.005)
        except KeyboardInterrupt:
            print("\nStopped by user.")

    jlink.rtt_stop()
    jlink.close()
    print(f"Wrote {lines_written} lines to {args.output}")


if __name__ == "__main__":
    main()
