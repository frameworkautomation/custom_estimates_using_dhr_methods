"""
Wrapper around DHR's main.py that records joint positions at every state execution.

Monkey-patches StateMachine.handle() to log robot.Joints() after each move.
Writes to robo_dk_output/joint_recordings/<task>_<timestamp>.csv

Usage (same as main.py, from clones/knitwear-cell):
    $env:ENV_MODE="local"
    py -3.12 ../../dhr_record.py

Then use dhr_cli.py to send commands. Every state execution gets logged.
Press Ctrl+C to stop — CSV is flushed after every write.
"""

import csv
import os
import sys
import time
from datetime import datetime

# Must run from clones/knitwear-cell
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
KNITWEAR_CELL = os.path.join(SCRIPT_DIR, "clones", "knitwear-cell")

if not os.path.isdir(KNITWEAR_CELL):
    print(f"[ERROR] knitwear-cell not found at {KNITWEAR_CELL}")
    sys.exit(1)

os.chdir(KNITWEAR_CELL)
sys.path.insert(0, KNITWEAR_CELL)
os.environ.setdefault("ENV_MODE", "local")

# Output directory
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "robo_dk_output", "joint_recordings")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Timestamp for this session
SESSION_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_PATH = os.path.join(OUTPUT_DIR, f"joints_{SESSION_TS}.csv")

# CSV setup
csv_file = open(CSV_PATH, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "timestamp", "elapsed_s", "state_name", "frame", "tool_name",
    "j1", "j2", "j3", "j4", "j5", "j6", "j7",
    "tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz",
])
csv_file.flush()

START_TIME = time.time()
MOVE_COUNT = 0

print(f"[RECORD] Logging joint positions to: {CSV_PATH}")


def patch_state_machine():
    """Monkey-patch StateMachine.handle() to log joints after each execution."""
    from src.main.robot.state_machine import StateMachine
    from robodk.robolink import Robolink
    from robodk.robomath import Pose_2_TxyzRxyz

    _original_handle = StateMachine.handle

    def _recording_handle(self, *args, **kwargs):
        global MOVE_COUNT
        result = _original_handle(self, *args, **kwargs)

        try:
            state = self._current_state
            state_name = state.__class__.__name__ if state else "unknown"
            frame = getattr(state, "frame", "") or ""
            tool_name = getattr(state, "tool_name", "") or ""

            # Get the robolink connection from the robot controller
            robot_ctrl = getattr(state, "robot_controller", None)
            if robot_ctrl and hasattr(robot_ctrl, "_robot"):
                robot = robot_ctrl._robot
            else:
                # Fallback: find robot via global module state
                import src.main.robot.robot as robot_mod
                robot = getattr(robot_mod, "_robot", None)

            if robot is not None:
                joints = robot.Joints().list()
                if len(joints) >= 7:
                    j1, j2, j3, j4, j5, j6, j7 = joints[:7]
                else:
                    j1 = j2 = j3 = j4 = j5 = j6 = j7 = 0

                # TCP pose
                try:
                    pose = robot.Pose()
                    xyzrxryrz = Pose_2_TxyzRxyz(pose)
                    tcp_x, tcp_y, tcp_z = xyzrxryrz[0], xyzrxryrz[1], xyzrxryrz[2]
                    tcp_rx, tcp_ry, tcp_rz = xyzrxryrz[3], xyzrxryrz[4], xyzrxryrz[5]
                except Exception:
                    tcp_x = tcp_y = tcp_z = tcp_rx = tcp_ry = tcp_rz = 0

                elapsed = time.time() - START_TIME
                now = datetime.now().isoformat(timespec="milliseconds")
                MOVE_COUNT += 1

                csv_writer.writerow([
                    now, f"{elapsed:.3f}", state_name, frame, tool_name,
                    f"{j1:.4f}", f"{j2:.4f}", f"{j3:.4f}", f"{j4:.4f}",
                    f"{j5:.4f}", f"{j6:.4f}", f"{j7:.4f}",
                    f"{tcp_x:.2f}", f"{tcp_y:.2f}", f"{tcp_z:.2f}",
                    f"{tcp_rx:.4f}", f"{tcp_ry:.4f}", f"{tcp_rz:.4f}",
                ])
                csv_file.flush()

                print(f"  [REC #{MOVE_COUNT}] {frame or state_name}: j7={j7:.1f}")

        except Exception as e:
            print(f"  [REC ERROR] {e}")

        return result

    StateMachine.handle = _recording_handle
    print("[RECORD] StateMachine.handle() patched for recording")


# Patch before importing main
patch_state_machine()

# Now run main.py
print("[RECORD] Starting DHR server with recording enabled...")
print(f"[RECORD] Use dhr_cli.py to send commands. All moves logged to CSV.")
print()

try:
    # Import main and call its entry point explicitly
    # (main.py guards with __name__ == "__main__", so import alone won't start it)
    from main import RobotService
    entry_point = RobotService()
    entry_point.main()
except KeyboardInterrupt:
    print("\n[RECORD] Stopped by user.")
except Exception as e:
    print(f"\n[RECORD] Server error: {e}")
    import traceback
    traceback.print_exc()
finally:
    csv_file.close()
    print(f"\n[RECORD] Session complete. {MOVE_COUNT} moves recorded to: {CSV_PATH}")
