"""
Build a RoboDK program from a joint recording CSV.

Reads a joint_recordings CSV, creates MoveJ instructions for each row,
then extracts the full interpolated trajectory via InstructionListJoints()
and displays it as ghost robots via ShowSequence().

Usage:
    py -3.12 csv_to_robodk_program.py                          # latest CSV
    py -3.12 csv_to_robodk_program.py path/to/joints.csv       # specific CSV
    py -3.12 csv_to_robodk_program.py --no-ghosts              # build program only
    py -3.12 csv_to_robodk_program.py --save-trajectory out.csv # save interpolated trajectory
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


def read_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser(description="Build RoboDK program from joint recording")
    parser.add_argument("csv_path", nargs="?", default=None)
    parser.add_argument("--no-ghosts", action="store_true", help="Don't display ghost robots")
    parser.add_argument("--save-trajectory", type=str, default=None,
                        help="Save interpolated trajectory to CSV")
    parser.add_argument("--mm-step", type=float, default=5.0, help="MoveL interpolation resolution (mm)")
    parser.add_argument("--deg-step", type=float, default=2.0, help="MoveJ interpolation resolution (deg)")
    parser.add_argument("--robodk-ip", type=str, default="localhost")
    parser.add_argument("--robodk-port", type=int, default=20502)
    args = parser.parse_args()

    csv_path = args.csv_path or find_latest_csv()
    if not csv_path or not os.path.exists(csv_path):
        print("[ERROR] No recording found.")
        sys.exit(1)

    rows = read_csv(csv_path)
    if not rows:
        print("[ERROR] CSV is empty.")
        sys.exit(1)

    print(f"[INFO] Loading {len(rows)} joint positions from {os.path.basename(csv_path)}")

    # Connect to RoboDK
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_PROGRAM
    from robodk.robolink import COLLISION_OFF
    from robodk.robolink import SEQUENCE_DISPLAY_OPTION_RESET, SEQUENCE_DISPLAY_ROBOT_JOINTS, SEQUENCE_DISPLAY_COLOR_TRANSPARENT
    from robodk.robomath import Mat

    rdk = Robolink(args.robodk_ip, args.robodk_port)
    print(f"[INFO] Connected to RoboDK")

    robot = rdk.ItemUserPick("Select robot", ITEM_TYPE_ROBOT)
    if not robot.Valid():
        print("[ERROR] No robot selected.")
        sys.exit(1)

    robot_name = robot.Name()
    n_joints = len(robot.Joints().list())
    print(f"[INFO] Robot: {robot_name} ({n_joints} joints)")

    # Delete old targets/program if they exist
    from robodk.robolink import ITEM_TYPE_TARGET, ITEM_TYPE_FRAME
    prog_name = "RecordedTrajectory"
    targets_frame_name = "RecordedTargets"

    old_prog = rdk.Item(prog_name, ITEM_TYPE_PROGRAM)
    if old_prog.Valid():
        old_prog.Delete()
    old_frame = rdk.Item(targets_frame_name, ITEM_TYPE_FRAME)
    if old_frame.Valid():
        old_frame.Delete()

    # Create parent frame for targets
    targets_frame = rdk.AddFrame(targets_frame_name)

    # Create program
    prog = rdk.AddProgram(prog_name, robot)
    prog.setRunType(0)  # simulation mode

    # Add MoveJ with proper joint targets for each row
    for i, row in enumerate(rows):
        joints = [
            float(row["j1"]), float(row["j2"]), float(row["j3"]),
            float(row["j4"]), float(row["j5"]), float(row["j6"]),
            float(row["j7"]),
        ]
        while len(joints) < n_joints:
            joints.append(0.0)
        joints = joints[:n_joints]

        # Create a joint target
        target = rdk.AddTarget(f"T{i:04d}", targets_frame, robot)
        target.setAsJointTarget()
        target.setJoints(joints)
        prog.MoveJ(target)

    print(f"[INFO] Program '{prog_name}' created with {len(rows)} MoveJ instructions")

    # Extract interpolated trajectory
    print(f"[INFO] Extracting interpolated trajectory (mm_step={args.mm_step}, deg_step={args.deg_step})...")
    status_msg, joint_list, status_code = prog.InstructionListJoints(
        mm_step=args.mm_step,
        deg_step=args.deg_step,
        collision_check=COLLISION_OFF,
        flags=1,  # include timestamps
        time_step=0.05,
    )

    if status_code < 0:
        print(f"[WARN] Program has issues: {status_msg}")

    # joint_list is a Mat (2D matrix)
    n_interp_rows = joint_list.size(0) if hasattr(joint_list, 'size') else len(joint_list)
    print(f"[INFO] Interpolated trajectory: {n_interp_rows} points")

    # Save interpolated trajectory if requested
    if args.save_trajectory:
        with open(args.save_trajectory, "w", newline="") as f:
            writer = csv.writer(f)
            header = [f"j{i+1}" for i in range(n_joints)] + ["error", "mm_step", "deg_step", "move_id", "time"]
            writer.writerow(header)
            for row_idx in range(n_interp_rows):
                row_data = joint_list[row_idx]
                if hasattr(row_data, 'list'):
                    row_data = row_data.list()
                elif hasattr(row_data, '__iter__'):
                    row_data = list(row_data)
                writer.writerow([f"{v:.6f}" for v in row_data])
        print(f"[INFO] Interpolated trajectory saved to: {args.save_trajectory}")

    # Show ghost robots — use the CSV joints directly as fallback if
    # InstructionListJoints returned too few points
    if not args.no_ghosts:
        print(f"[INFO] Displaying ghost robots in RoboDK...")
        display_flags = SEQUENCE_DISPLAY_OPTION_RESET | SEQUENCE_DISPLAY_ROBOT_JOINTS | SEQUENCE_DISPLAY_COLOR_TRANSPARENT

        if n_interp_rows > len(rows) // 2:
            # Interpolated trajectory looks good, use it
            robot.ShowSequence(joint_list, display_flags, 3600 * 1000)
            print(f"[INFO] Ghost robots displayed from interpolated trajectory ({n_interp_rows} points)")
        else:
            # Fallback: build matrix from CSV joints directly
            print(f"[INFO] Interpolation had issues ({n_interp_rows} pts), using CSV joints directly...")
            from robodk.robomath import Mat as RMat
            csv_joints = []
            for row in rows:
                j = [
                    float(row["j1"]), float(row["j2"]), float(row["j3"]),
                    float(row["j4"]), float(row["j5"]), float(row["j6"]),
                    float(row["j7"]),
                ]
                while len(j) < n_joints:
                    j.append(0.0)
                csv_joints.append(j[:n_joints])
            robot.ShowSequence(RMat(csv_joints), display_flags, 3600 * 1000)
            print(f"[INFO] Ghost robots displayed from CSV ({len(csv_joints)} positions)")

    print(f"[DONE]")


if __name__ == "__main__":
    main()
