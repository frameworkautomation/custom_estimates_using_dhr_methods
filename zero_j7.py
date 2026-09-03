"""
Copy a joint recording CSV with j7 set to 0.

This removes the rail translation so all arm poses are shown relative to
the robot base. Use the output with csv_to_robodk_program.py to see the
arm's swept volume in base-relative space.

Usage:
    py -3.12 zero_j7.py                          # latest CSV
    py -3.12 zero_j7.py path/to/joints.csv       # specific CSV
    py -3.12 zero_j7.py --output custom_name.csv  # custom output name
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
    # Skip any _j7zero files
    csvs = [c for c in csvs if "_j7zero" not in c.name]
    return str(csvs[-1]) if csvs else None


def main():
    parser = argparse.ArgumentParser(description="Zero out j7 in a joint recording")
    parser.add_argument("csv_path", nargs="?", default=None)
    parser.add_argument("--output", type=str, default=None, help="Output filename")
    args = parser.parse_args()

    csv_path = args.csv_path or find_latest_csv()
    if not csv_path or not os.path.exists(csv_path):
        print("[ERROR] No recording found.")
        sys.exit(1)

    # Generate output path
    if args.output:
        out_path = args.output
    else:
        base = Path(csv_path)
        out_path = str(base.parent / (base.stem + "_j7zero" + base.suffix))

    # Read, zero j7, write
    count = 0
    with open(csv_path) as fin, open(out_path, "w", newline="") as fout:
        reader = csv.DictReader(fin)
        fieldnames = reader.fieldnames
        writer = csv.DictWriter(fout, fieldnames=fieldnames)
        writer.writeheader()

        for row in reader:
            row["j7"] = "0.0000"
            # Also zero out TCP world coords since they're no longer valid
            # (they included j7 translation)
            row["tcp_x"] = "0.00"
            row["tcp_y"] = "0.00"
            row["tcp_z"] = "0.00"
            row["tcp_rx"] = "0.0000"
            row["tcp_ry"] = "0.0000"
            row["tcp_rz"] = "0.0000"
            writer.writerow(row)
            count += 1

    print(f"[DONE] {count} rows written with j7=0 to: {out_path}")
    print(f"       Use with: py -3.12 csv_to_robodk_program.py {out_path}")


if __name__ == "__main__":
    main()
