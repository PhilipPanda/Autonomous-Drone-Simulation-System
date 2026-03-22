#!/usr/bin/env python3

import sys
import csv
import argparse
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def load_csv(path: Path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) if k not in ("failsafe",) else v for k, v in row.items()})
    return rows


def extract(rows, key):
    return [r[key] for r in rows]


def plot(rows, output=None):
    t = extract(rows, "timestamp")
    px = extract(rows, "pos_x")
    py = extract(rows, "pos_y")
    pz = extract(rows, "pos_z")
    vx = extract(rows, "vel_x")
    vy = extract(rows, "vel_y")
    vz = extract(rows, "vel_z")
    roll = extract(rows, "roll")
    pitch = extract(rows, "pitch")
    yaw = extract(rows, "yaw")
    est_roll = extract(rows, "est_roll")
    est_pitch = extract(rows, "est_pitch")
    est_yaw = extract(rows, "est_yaw")
    imu_valid = extract(rows, "imu_valid")

    fig = plt.figure(figsize=(16, 12))
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.4, wspace=0.35)

    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(t, px, label="x")
    ax1.plot(t, py, label="y")
    ax1.plot(t, pz, label="z")
    ax1.set_title("Position (m)")
    ax1.set_xlabel("t (s)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, vx, label="vx")
    ax2.plot(t, vy, label="vy")
    ax2.plot(t, vz, label="vz")
    ax2.set_title("Velocity (m/s)")
    ax2.set_xlabel("t (s)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t, [r * 57.296 for r in roll], label="roll")
    ax3.plot(t, [p * 57.296 for p in pitch], label="pitch")
    ax3.plot(t, [y * 57.296 for y in yaw], label="yaw")
    ax3.set_title("True Attitude (deg)")
    ax3.set_xlabel("t (s)")
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t, [r * 57.296 for r in est_roll], label="est roll", linestyle="--")
    ax4.plot(t, [p * 57.296 for p in est_pitch], label="est pitch", linestyle="--")
    ax4.plot(t, [y * 57.296 for y in est_yaw], label="est yaw", linestyle="--")
    ax4.set_title("Estimated Attitude (deg)")
    ax4.set_xlabel("t (s)")
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    ax5 = fig.add_subplot(gs[2, 0])
    ax5.plot(px, py)
    ax5.set_title("Ground Track (m)")
    ax5.set_xlabel("x (m)")
    ax5.set_ylabel("y (m)")
    ax5.set_aspect("equal", adjustable="box")
    ax5.grid(True, alpha=0.3)

    ax6 = fig.add_subplot(gs[2, 1])
    ax6.plot(t, imu_valid, drawstyle="steps-post")
    ax6.set_title("IMU Valid")
    ax6.set_xlabel("t (s)")
    ax6.set_ylim(-0.1, 1.1)
    ax6.grid(True, alpha=0.3)

    if output:
        plt.savefig(output, dpi=150, bbox_inches="tight")
        print(f"Saved to {output}")
    else:
        plt.show()


def print_stats(rows):
    t = extract(rows, "timestamp")
    pz = extract(rows, "pos_z")
    imu_valid = extract(rows, "imu_valid")
    dropouts = sum(1 for v in imu_valid if v == 0.0)

    print(f"Records     : {len(rows)}")
    print(f"Duration    : {t[-1]:.2f} s")
    print(f"Max altitude: {max(pz):.2f} m")
    print(f"Min altitude: {min(pz):.2f} m")
    print(f"IMU dropouts: {dropouts} samples")


def main():
    parser = argparse.ArgumentParser(description="Plot adsim CSV log")
    parser.add_argument("log", help="Path to CSV log file")
    parser.add_argument("-o", "--output", help="Save plot to file instead of displaying")
    parser.add_argument("--stats", action="store_true", help="Print statistics only")
    args = parser.parse_args()

    path = Path(args.log)
    if not path.exists():
        print(f"File not found: {path}", file=sys.stderr)
        sys.exit(1)

    rows = load_csv(path)

    print_stats(rows)

    if args.stats:
        return

    if not HAS_MPL:
        print("matplotlib not available. Install it with: pip install matplotlib")
        sys.exit(1)

    plot(rows, args.output)


if __name__ == "__main__":
    main()
