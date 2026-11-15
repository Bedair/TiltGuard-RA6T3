#!/usr/bin/env python3
"""
Plot a TiltGuard CSV captured by capture.py - one figure per armed run.

The firmware writes a header line at every arm, so a capture holds one or
more runs. Each run gets five stacked panels against time:
  1. pitch angle and setpoint, with the +/-2 deg logic-check band and the
     +/-25 deg tilt-fault limit
  2. gyro_x pitch rate
  3. the law's terms in stock 8-bit units (bal, spd, turn; +/-255 = full)
  4. the driver command per wheel as signed duty (FWD +, REV -), permille,
     with red marks where the logic check would flag a direction error
     (|ang - sp| > 2 deg and a wheel not commanded in the direction of
     ang - sp). The firmware's own check also skips samples where D/turn
     outweigh half the P term; this plot can't (it doesn't know kp/kd), so
     marks there can be legitimate PD braking - compare with sign_err.
  5. encoder edges per 40 ms window (unsigned)

The title says ACTIVE or DRY-RUN. --run N plots only run N (1-based);
--save writes PNGs (out.png -> out_run1.png, ...) instead of showing.
"""
import argparse
import io
import os

import matplotlib.pyplot as plt
import pandas as pd

CSV_HEADER = "t_ms,ang,sp,gyro,bal,spd,turn,dirL,dutyL,dirR,dutyR,encL,encR,motL,motR,mode"
TILT_FAULT_LIMIT_DEG = 25.0  # firmware/src/safety/safety.h SAFETY_TILT_LIMIT_DEG
CHECK_MIN_ERROR_DEG = 2.0  # firmware/src/control/balance.h BALANCE_CHECK_MIN_ERROR_DEG
PWM_LIMIT_8BIT = 255
DIRECTION_SIGN = {"FWD": 1, "REV": -1, "STOP": 0}


def read_runs(path):
    """Split the capture at every header line into one DataFrame per run."""
    runs, current = [], []
    with open(path, encoding="ascii", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line == CSV_HEADER:
                if current:
                    runs.append(current)
                current = []
            else:
                current.append(line)
    if current:
        runs.append(current)
    return [pd.read_csv(io.StringIO(CSV_HEADER + "\n" + "\n".join(rows))) for rows in runs]


def plot_run(df, number, save_path):
    t = (df["t_ms"] - df["t_ms"].iloc[0]) / 1000.0
    err = df["ang"] - df["sp"]
    signed_l = df["dirL"].map(DIRECTION_SIGN) * df["dutyL"]
    signed_r = df["dirR"].map(DIRECTION_SIGN) * df["dutyR"]

    expected = err.apply(lambda e: 1 if e > 0 else -1)
    checked = err.abs() > CHECK_MIN_ERROR_DEG
    mismatch = checked & ((df["dirL"].map(DIRECTION_SIGN) != expected) |
                          (df["dirR"].map(DIRECTION_SIGN) != expected))

    mode = "/".join(sorted(df["mode"].unique()))
    fig, axes = plt.subplots(5, 1, sharex=True, figsize=(11, 12))
    fig.suptitle(f"run {number} - {mode} - {len(df)} samples, "
                 f"{int(checked.sum())} with |ang-sp| > {CHECK_MIN_ERROR_DEG:g} deg, "
                 f"{int(mismatch.sum())} direction mismatches, "
                 f"NONE (driven, wheel didn't turn) L={int((df['motL'] == 'NONE').sum())} "
                 f"R={int((df['motR'] == 'NONE').sum())}")

    ax = axes[0]
    ax.plot(t, df["ang"], label="ang (> 0 = tilted forward)")
    ax.plot(t, df["sp"], color="k", linestyle="--", linewidth=1, label="sp")
    ax.fill_between(t, df["sp"] - CHECK_MIN_ERROR_DEG, df["sp"] + CHECK_MIN_ERROR_DEG,
                    color="tab:gray", alpha=0.15, linewidth=0, label="not checked (|ang-sp| <= 2)")
    ax.axhline(TILT_FAULT_LIMIT_DEG, color="r", linestyle=":", linewidth=1, label="tilt fault limit")
    ax.axhline(-TILT_FAULT_LIMIT_DEG, color="r", linestyle=":", linewidth=1)
    ax.set_ylabel("deg")
    ax.legend(loc="upper right", fontsize="small")

    ax = axes[1]
    ax.plot(t, df["gyro"], label="gyro_x")
    ax.set_ylabel("deg/s")
    ax.legend(loc="upper right", fontsize="small")

    ax = axes[2]
    ax.plot(t, df["bal"], label="bal (PD)", linewidth=1.2)
    ax.plot(t, df["spd"], label="spd (speed PI)", linewidth=1)
    ax.plot(t, df["turn"], label="turn", linewidth=1)
    ax.axhline(PWM_LIMIT_8BIT, color="r", linestyle=":", linewidth=1, label="clamp +/-255")
    ax.axhline(-PWM_LIMIT_8BIT, color="r", linestyle=":", linewidth=1)
    ax.set_ylabel("stock 8-bit units")
    ax.legend(loc="upper right", fontsize="small", ncol=2)

    ax = axes[3]
    ax.plot(t, signed_l, label="L (FWD +, REV -)", linewidth=1.2)
    ax.plot(t, signed_r, label="R", linewidth=1.2, linestyle="--")
    if mismatch.any():
        ax.scatter(t[mismatch], signed_l[mismatch], color="r", s=12, zorder=3,
                   label="direction != sign(ang - sp)")
    ax.set_ylabel("duty permille")
    ax.legend(loc="upper right", fontsize="small")

    ax = axes[4]
    ax.plot(t, df["encL"], label="encL")
    ax.plot(t, df["encR"], label="encR", linestyle="--")
    ax.set_ylabel("edges / 40 ms")
    ax.set_xlabel("time since arm (s)")
    ax.legend(loc="upper right", fontsize="small")

    for ax in axes:
        ax.grid(True, alpha=0.3)

    fig.tight_layout()

    if save_path:
        root, ext = os.path.splitext(save_path)
        out = f"{root}_run{number}{ext or '.png'}"
        fig.savefig(out)
        plt.close(fig)
        print(f"Saved {out}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv_file", help="CSV file written by capture.py")
    parser.add_argument("--run", type=int, default=None, help="Plot only this run (1-based)")
    parser.add_argument("--save", help="Save PNGs (<name>_run<N>.png) instead of showing them")
    args = parser.parse_args()

    runs = read_runs(args.csv_file)
    if not runs:
        raise SystemExit("No samples - the firmware only sends rows while armed (press S1 in READY).")

    numbers = range(1, len(runs) + 1) if args.run is None else [args.run]
    for number in numbers:
        if not 1 <= number <= len(runs):
            raise SystemExit(f"Run {number} not in this file ({len(runs)} runs).")
        plot_run(runs[number - 1], number, args.save)

    if not args.save:
        plt.show()


if __name__ == "__main__":
    main()
