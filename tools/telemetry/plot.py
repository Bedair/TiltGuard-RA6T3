#!/usr/bin/env python3
"""
Plot a TiltGuard telemetry CSV captured by capture.py.

Shows filtered pitch angle, gyro rate, motor command, encoder speed, and
the safety state machine, all against time (derived from the 1 kHz tick
column).
"""
import argparse

import matplotlib.pyplot as plt
import pandas as pd

SAFETY_STATE_NAMES = {0: "INIT", 1: "CALIBRATING", 2: "READY", 3: "ACTIVE", 4: "FAULT"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", help="CSV file written by capture.py")
    parser.add_argument("--save", help="Save the figure to this path instead of showing it interactively")
    args = parser.parse_args()

    df = pd.read_csv(args.csv_file)
    t = df["tick"] / 1000.0  # 1 kHz ticks -> seconds

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10))

    axes[0].plot(t, df["angle_centideg"] / 100.0, label="filtered pitch angle")
    axes[0].axhline(35.0, color="r", linestyle=":", linewidth=1, label="tilt fault limit")
    axes[0].axhline(-35.0, color="r", linestyle=":", linewidth=1)
    axes[0].set_ylabel("deg")
    axes[0].legend(loc="upper right", fontsize="small")

    axes[1].plot(t, df["gyro_centidps"] / 100.0, label="gyro_x rate")
    axes[1].set_ylabel("deg/s")
    axes[1].legend(loc="upper right", fontsize="small")

    axes[2].plot(t, df["motor_L_permille"], label="motor L command")
    axes[2].plot(t, df["motor_R_permille"], label="motor R command")
    axes[2].plot(t, df["enc_L_counts_per_5ms"], label="enc L speed", linestyle="--")
    axes[2].plot(t, df["enc_R_counts_per_5ms"], label="enc R speed", linestyle="--")
    axes[2].set_ylabel("permille / counts per 5ms")
    axes[2].legend(loc="upper right", fontsize="small")

    axes[3].step(t, df["safety_state"], where="post")
    axes[3].set_yticks(list(SAFETY_STATE_NAMES.keys()))
    axes[3].set_yticklabels(list(SAFETY_STATE_NAMES.values()))
    axes[3].set_ylabel("safety state")
    axes[3].set_xlabel("time (s)")

    fig.tight_layout()

    if args.save:
        fig.savefig(args.save)
        print(f"Saved to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
