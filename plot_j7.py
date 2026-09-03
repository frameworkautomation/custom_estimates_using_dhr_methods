"""
Read a joint recording CSV and plot j7 trajectory.

Usage:
    py -3.12 plot_j7.py                           # uses most recent recording
    py -3.12 plot_j7.py path/to/joints_*.csv       # specific file
    py -3.12 plot_j7.py --list                      # list available recordings
"""

import argparse
import csv
import os
import sys
from pathlib import Path

RECORDINGS_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "robo_dk_output", "joint_recordings"
)


def find_latest_csv():
    if not os.path.isdir(RECORDINGS_DIR):
        return None
    csvs = sorted(Path(RECORDINGS_DIR).glob("joints_*.csv"))
    return str(csvs[-1]) if csvs else None


def list_recordings():
    if not os.path.isdir(RECORDINGS_DIR):
        print("No recordings directory found.")
        return
    csvs = sorted(Path(RECORDINGS_DIR).glob("joints_*.csv"))
    if not csvs:
        print("No recordings found.")
        return
    print(f"\nRecordings in {RECORDINGS_DIR}:\n")
    for f in csvs:
        size = f.stat().st_size
        with open(f) as fh:
            lines = sum(1 for _ in fh) - 1  # minus header
        print(f"  {f.name}  ({lines} moves, {size:,} bytes)")
    print()


def read_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def print_summary(rows, path):
    if not rows:
        print("No data in recording.")
        return

    j7_values = [float(r["j7"]) for r in rows]
    frames = [r["frame"] or r["state_name"] for r in rows]
    elapsed = [float(r["elapsed_s"]) for r in rows]

    j7_min = min(j7_values)
    j7_max = max(j7_values)
    j7_range = j7_max - j7_min

    min_idx = j7_values.index(j7_min)
    max_idx = j7_values.index(j7_max)

    print(f"\n{'=' * 60}")
    print(f"  J7 Trajectory Report")
    print(f"  File: {os.path.basename(path)}")
    print(f"  Moves: {len(rows)}")
    print(f"  Duration: {elapsed[-1]:.1f}s")
    print(f"{'=' * 60}")
    print(f"  J7 min:   {j7_min:>10.1f} mm  (at {frames[min_idx]})")
    print(f"  J7 max:   {j7_max:>10.1f} mm  (at {frames[max_idx]})")
    print(f"  J7 range: {j7_range:>10.1f} mm")
    print(f"{'=' * 60}")

    # Show j7 at each unique frame
    print(f"\n  {'#':>4}  {'Elapsed':>8}  {'J7':>10}  Frame")
    print(f"  {'─' * 4}  {'─' * 8}  {'─' * 10}  {'─' * 40}")
    for i, row in enumerate(rows):
        j7 = float(row["j7"])
        t = float(row["elapsed_s"])
        frame = row["frame"] or row["state_name"]
        print(f"  {i + 1:>4}  {t:>7.1f}s  {j7:>9.1f}  {frame}")

    print()


def plot_j7(rows, path):
    """Plot j7 over move index. Uses matplotlib if available, ASCII fallback otherwise."""
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt

        j7_values = [float(r["j7"]) for r in rows]
        indices = list(range(1, len(j7_values) + 1))
        elapsed = [float(r["elapsed_s"]) for r in rows]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=False)

        # Plot 1: j7 vs move index
        ax1.plot(indices, j7_values, "b-o", markersize=3, linewidth=1)
        ax1.set_xlabel("Move #")
        ax1.set_ylabel("J7 position (mm)")
        ax1.set_title(f"J7 Trajectory — {os.path.basename(path)}")
        ax1.axhline(y=min(j7_values), color="r", linestyle="--", alpha=0.5,
                     label=f"min={min(j7_values):.0f}")
        ax1.axhline(y=max(j7_values), color="g", linestyle="--", alpha=0.5,
                     label=f"max={max(j7_values):.0f}")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: j7 vs elapsed time
        ax2.plot(elapsed, j7_values, "b-o", markersize=3, linewidth=1)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("J7 position (mm)")
        ax2.set_title("J7 vs Time")
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    except ImportError:
        print("\n  [matplotlib not installed — showing ASCII plot]")
        print("  Install with: py -3.12 -m pip install matplotlib\n")
        ascii_plot(rows)
    except Exception as e:
        print(f"\n  [Plot error: {e} — showing ASCII plot]\n")
        ascii_plot(rows)


def ascii_plot(rows):
    """Simple ASCII bar chart of j7 values."""
    j7_values = [float(r["j7"]) for r in rows]
    frames = [r["frame"] or r["state_name"] for r in rows]

    j7_min = min(j7_values)
    j7_max = max(j7_values)
    span = j7_max - j7_min if j7_max != j7_min else 1
    width = 50

    for i, (j7, frame) in enumerate(zip(j7_values, frames)):
        bar_len = int((j7 - j7_min) / span * width)
        bar = "█" * bar_len
        print(f"  {i + 1:>3} {j7:>8.0f} |{bar:<{width}}| {frame[:30]}")


def main():
    parser = argparse.ArgumentParser(description="Plot j7 trajectory from recording")
    parser.add_argument("csv_path", nargs="?", default=None, help="Path to CSV file")
    parser.add_argument("--list", action="store_true", help="List available recordings")
    parser.add_argument("--no-plot", action="store_true", help="Summary only, no graph")
    args = parser.parse_args()

    if args.list:
        list_recordings()
        return

    csv_path = args.csv_path or find_latest_csv()
    if not csv_path or not os.path.exists(csv_path):
        print("[ERROR] No recording found. Run dhr_record.py first, then send commands via dhr_cli.py")
        sys.exit(1)

    rows = read_csv(csv_path)
    print_summary(rows, csv_path)

    if not args.no_plot:
        plot_j7(rows, csv_path)


if __name__ == "__main__":
    main()
