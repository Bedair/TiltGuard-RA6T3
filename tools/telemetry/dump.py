#!/usr/bin/env python3
"""
Read TiltGuard's black-box recording (the last 10 s of the most recent armed
run), save it as CSV, and plot it.

The firmware keeps the recording in RAM that survives a reset (S2) and a
debugger attach, but NOT a power loss - so after an untethered run, plug
the laptop in without cutting the board's power, then run:

    python dump.py                     # -> blackbox_<date>_<time>.csv + plot window
    python dump.py run7.csv --save run7.png
    python dump.py --plot-only run7.csv

It connects through the on-board J-Link (no reset, no flashing), sends the
"dump" console command on RTT down-channel 0 and collects the CSV from
channel 1. Don't start an e2 studio debug session first: that reflashes the
board, and the flash loader can overwrite the recording.

File format (firmware/src/telemetry/blackbox.c):
    # blackbox build=<git hash>[-dirty] <date> <time>
    # run reason=<S1|tilt|fault|reset> faults=0x.. records=N armed_ms=.. period_ms=10 mode=.. speed_loop=.. arm_angle=..
    # params kp=.. kd=.. sp=.. skp=.. ski=.. tkd=..
    t_ms,ang,gyro,pwmL,pwmR,bal,spd,encL,encR
    ...                                  100 Hz rows, oldest first
    # end records=N
Columns: t_ms since arm; ang deg (> 0 = tilted forward); gyro deg/s; pwmL/pwmR
signed driver duty, permille (+ = FWD; planned only, in DRY-RUN); bal = kp*e +
kd*gyro and spd = applied speed-loop term, stock 8-bit units (pwm = bal - spd
-/+ turn, +/-255 = full); encL/encR signed encoder position since arm,
counts (660 per wheel revolution; int16, unwrapped here when plotting).

Requires pylink-square, pandas and matplotlib (pip install -r requirements.txt).
"""
import argparse
import datetime
import io
import sys
import time

import pylink

DEFAULT_DEVICE = "R7FA6T3BB"
DATA_CHANNEL = 1
CONSOLE_CHANNEL = 0
COLUMNS = "t_ms,ang,gyro,pwmL,pwmR,bal,spd,encL,encR"
TILT_LIMIT_DEG = 25.0  # firmware/src/safety/safety.h SAFETY_TILT_LIMIT_DEG
STANDUP_LIMIT_DEG = 45.0  # SAFETY_STANDUP_TILT_LIMIT_DEG (first 2 s after arm)
STANDUP_MS = 2000  # SAFETY_STANDUP_TICKS
COUNTS_PER_REV = 660


def wait_for_rtt(jlink, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            if jlink.rtt_get_num_up_buffers() > DATA_CHANNEL:
                return True
        except pylink.errors.JLinkRTTException:
            pass
        time.sleep(0.05)
    return False


def read_text(jlink, channel):
    data = jlink.rtt_read(channel, 1024)
    return bytes(data).decode("ascii", errors="replace") if data else ""


def fetch(device, timeout_s):
    """Send 'dump', return the dump's lines (channel 1)."""
    jlink = pylink.JLink()
    jlink.open()
    jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)
    jlink.connect(device)
    jlink.rtt_start()
    try:
        if not wait_for_rtt(jlink):
            sys.exit("Timed out waiting for the RTT control block (is the firmware running?)")

        # Drop anything already queued on channel 1 (e.g. live CSV from a run).
        quiet_since = time.monotonic()
        while time.monotonic() - quiet_since < 0.3:
            if read_text(jlink, DATA_CHANNEL):
                quiet_since = time.monotonic()
            time.sleep(0.01)

        jlink.rtt_write(CONSOLE_CHANNEL, list(b"dump\r\n"))
        print("sent 'dump', waiting for the CSV on channel 1 ...")

        lines, partial, console = [], "", ""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            # Echo the firmware's replies about the dump (refused / done).
            console += read_text(jlink, CONSOLE_CHANNEL)
            while "\n" in console:
                line, console = console.split("\n", 1)
                if line.startswith(("dump", "blackbox")):
                    print("firmware:", line.strip())

            text = read_text(jlink, DATA_CHANNEL)
            if not text:
                time.sleep(0.005)
                continue
            deadline = time.monotonic() + timeout_s  # Still arriving.
            partial += text
            while "\n" in partial:
                line, partial = partial.split("\n", 1)
                line = line.strip("\r")
                if not line:
                    continue
                if not lines and not line.startswith("# blackbox"):
                    continue  # Not part of the dump.
                lines.append(line)
                if line.startswith("# end") or line.startswith("# dump aborted"):
                    return lines
        sys.exit("No complete dump received (see the firmware's 'dump ...' reply above, "
                 "or type 'dump' in RTT Viewer to see why).")
    finally:
        jlink.rtt_stop()
        jlink.close()


def unwrap_int16(series):
    out, offset, prev = [], 0, None
    for value in series:
        if prev is not None:
            step = value - prev
            if step > 32767:
                offset -= 65536
            elif step < -32768:
                offset += 65536
        out.append(value + offset)
        prev = value
    return out


def plot(path, save):
    import matplotlib.pyplot as plt
    import pandas as pd

    with open(path, encoding="ascii", errors="replace") as f:
        text = f.read()
    meta = [line[2:] for line in text.splitlines() if line.startswith("# ")]
    df = pd.read_csv(io.StringIO(text), comment="#")
    if df.empty:
        sys.exit(f"{path}: no records")
    t = df["t_ms"] / 1000.0

    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(12, 10))
    fig.suptitle("\n".join(meta[:3]), fontsize=8, ha="left", x=0.01)

    ax[0].plot(t, df["ang"], label="ang")
    for limit in (TILT_LIMIT_DEG, -TILT_LIMIT_DEG):
        ax[0].axhline(limit, color="red", lw=0.8, ls="--")
    for limit in (STANDUP_LIMIT_DEG, -STANDUP_LIMIT_DEG):
        ax[0].hlines(limit, 0, STANDUP_MS / 1000.0, color="orange", lw=0.8, ls="--")
    ax[0].axhline(0, color="grey", lw=0.5)
    ax[0].set_ylabel("angle, deg\n(>0 = forward)")
    ax[0].legend(loc="upper right", fontsize=8)

    ax[1].plot(t, df["pwmL"], label="pwm L")
    ax[1].plot(t, df["pwmR"], label="pwm R")
    ax[1].axhline(0, color="grey", lw=0.5)
    ax[1].set_ylabel("driver duty,\npermille (+ = FWD)")
    ax[1].legend(loc="upper right", fontsize=8)

    ax[2].plot(t, unwrap_int16(df["encL"]), label="enc L")
    ax[2].plot(t, unwrap_int16(df["encR"]), label="enc R")
    ax[2].set_ylabel(f"position since arm,\ncounts ({COUNTS_PER_REV}/rev)")
    ax[2].legend(loc="upper right", fontsize=8)

    ax[3].plot(t, df["bal"], label="bal (kp*e + kd*gyro)")
    ax[3].plot(t, df["spd"], label="spd (applied)")
    for limit in (255, -255):
        ax[3].axhline(limit, color="grey", lw=0.5, ls="--")
    ax[3].set_ylabel("terms,\nstock 8-bit units")
    ax[3].set_xlabel("time since arm, s")
    ax[3].legend(loc="upper right", fontsize=8)

    for a in ax:
        a.grid(True, alpha=0.3)
    fig.tight_layout(rect=(0, 0, 1, 0.93))

    if save:
        fig.savefig(save, dpi=120)
        print(f"plot saved to {save}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("output", nargs="?", help="CSV file to write (default: blackbox_<date>_<time>.csv)")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="J-Link target device (default: %(default)s)")
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds without data before giving up")
    parser.add_argument("--save", metavar="PNG", help="write the plot to this file instead of showing it")
    parser.add_argument("--no-plot", action="store_true", help="only save the CSV")
    parser.add_argument("--plot-only", metavar="CSV", help="plot an existing dump, don't connect")
    args = parser.parse_args()

    if args.plot_only:
        plot(args.plot_only, args.save)
        return

    output = args.output or datetime.datetime.now().strftime("blackbox_%Y%m%d_%H%M%S.csv")
    lines = fetch(args.device, args.timeout)
    with open(output, "w", newline="") as f:
        f.write("\n".join(lines) + "\n")
    rows = sum(1 for line in lines if line and line[0].isdigit())
    print(f"wrote {rows} records to {output}")
    for line in lines[:3]:
        print(" ", line)
    if lines[-1].startswith("# dump aborted"):
        sys.exit("dump aborted: the robot was armed during the dump")

    if not args.no_plot:
        plot(output, args.save)


if __name__ == "__main__":
    main()
