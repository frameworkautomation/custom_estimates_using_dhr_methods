"""
save_joint_position.py

Reads the current robot joint positions from RoboDK and saves them as a named
waypoint in robo_dk_output/path_config.yaml under the `waypoints:` section.

Workflow:
  1. Jog the robot in RoboDK to the desired pose.
  2. Run this script with a name for the waypoint.
  3. Edit path_config.yaml to add the name to `routing_candidates:` if needed.
  4. Re-run check_collision_free_paths.py to test edges.

Usage:
    python robodk_code/save_joint_position.py
    python robodk_code/save_joint_position.py --name transport_j7_0
    python robodk_code/save_joint_position.py --name curtain_safe_machine_1 --robodk-ip 172.23.208.1
    python robodk_code/save_joint_position.py --name home --print-only

If --name is omitted, a tkinter dialog pops up asking for the name.
"""

import sys
import os
import argparse
import re

sys.path.append("C:/RoboDK/Python")

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
CONFIG_PATH = os.path.join(REPO_ROOT, "robo_dk_output", "path_config.yaml")
ROBOT_NAME = "Fanuc R2000iC 125L"


def connect(ip):
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    rdk = Robolink(ip)
    robot = rdk.Item(ROBOT_NAME, ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError(f"Robot '{ROBOT_NAME}' not found in station.")
    return rdk, robot


def get_joints(robot):
    joints = robot.Joints().list()
    return [round(float(j), 4) for j in joints]


def format_joints(joints):
    return "[" + ", ".join(f"{j:.4f}" for j in joints) + "]"


def waypoint_exists(name):
    """Check if a waypoint with this name already exists in path_config.yaml."""
    if not os.path.exists(CONFIG_PATH):
        return False
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        content = f.read()
    # Match "  name:" as a YAML mapping key under waypoints
    return bool(re.search(rf'^\s+{re.escape(name)}\s*:', content, re.MULTILINE))


def append_waypoint(name, joints):
    """Append a new joint waypoint to path_config.yaml."""
    if not os.path.exists(CONFIG_PATH):
        raise FileNotFoundError(f"path_config.yaml not found at {CONFIG_PATH}")

    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        content = f.read()

    entry = f"\n  {name}:\n    joints: {format_joints(joints)}\n    source: human\n"

    # Insert after the `waypoints:` section header (before routing_candidates)
    if "routing_candidates:" in content:
        content = content.replace("routing_candidates:", entry + "routing_candidates:", 1)
    elif "waypoints:" in content:
        # Append at end of waypoints block — find last waypoint entry
        content += entry
    else:
        raise RuntimeError("Could not find 'waypoints:' section in path_config.yaml")

    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        f.write(content)


def _report_collision_status(name, repo_root):
    """Look up waypoint name in waypoint_collisions.json and report status.

    Also writes collision_checked: true/false into the waypoint entry in
    path_config.yaml if the JSON has data for this waypoint.
    """
    import json

    collisions_path = os.path.join(repo_root, "robo_dk_output", "waypoint_collisions.json")

    if not os.path.exists(collisions_path):
        print("  Collision: not yet checked — run check_waypoint_collisions.py")
        return

    with open(collisions_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    waypoints = data.get("waypoints", {})
    if name not in waypoints:
        print("  Collision: not yet checked — run check_waypoint_collisions.py")
        return

    entry = waypoints[name]
    collision = entry.get("collision", None)

    if collision is False:
        print("  Collision: CLEAR")
        checked_value = "true"
    elif collision is True:
        colliding = entry.get("colliding_items", [])
        print(f"  Collision: DETECTED — {colliding}")
        checked_value = "false"
    else:
        print("  Collision: not yet checked — run check_waypoint_collisions.py")
        return

    # Write collision_checked field into the waypoint entry in path_config.yaml
    config_path = os.path.join(repo_root, "robo_dk_output", "path_config.yaml")
    if not os.path.exists(config_path):
        return

    with open(config_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Find the waypoint block and insert collision_checked after the source: line.
    # Pattern: the waypoint name followed (eventually) by "source: human" — insert after.
    import re as _re
    # Replace `source: human` within this waypoint's block (first occurrence after the name key)
    marker = f"  {name}:\n"
    name_pos = content.find(marker)
    if name_pos == -1:
        return  # waypoint block not found — don't corrupt the file

    # Search for `source: human` starting from the waypoint name position
    source_pattern = _re.compile(r"(    source: human\n)", _re.MULTILINE)
    match = source_pattern.search(content, name_pos)
    if match and "collision_checked:" not in content[name_pos:match.end() + 50]:
        insert_pos = match.end()
        content = (
            content[:insert_pos]
            + f"    collision_checked: {checked_value}\n"
            + content[insert_pos:]
        )
        with open(config_path, "w", encoding="utf-8") as f:
            f.write(content)


def ask_name_dialog(joints_str):
    """Show a dialog to enter the waypoint name. Returns name or None if cancelled.

    Uses PowerShell InputBox (works from WSL), falls back to tkinter.
    """
    # WSL: call powershell.exe on the Windows side
    if os.path.exists("/proc/sys/fs/binfmt_misc/WSLInterop"):
        import subprocess
        ps_cmd = (
            "Add-Type -AssemblyName 'Microsoft.VisualBasic'; "
            f"[Microsoft.VisualBasic.Interaction]::InputBox("
            f"'Joints: {joints_str}`n`nWaypoint name:', 'Save Waypoint', '')"
        )
        result = subprocess.run(
            ["powershell.exe", "-Command", ps_cmd],
            capture_output=True, text=True,
        )
        name = result.stdout.strip()
        return name if name else None

    # Non-WSL (Windows or Linux with display): try tkinter
    try:
        import tkinter as tk
        from tkinter import simpledialog
        root = tk.Tk()
        root.withdraw()
        root.lift()
        root.attributes("-topmost", True)
        name = simpledialog.askstring(
            "Save Waypoint",
            f"Joints: {joints_str}\n\nWaypoint name:",
            parent=root,
        )
        root.destroy()
        return name.strip() if name else None
    except Exception as e:
        print(f"[WARN] Dialog failed ({e}). Pass --name explicitly.")
        return None


def main():
    parser = argparse.ArgumentParser(description="Save current robot joint position to path_config.yaml")
    parser.add_argument("--name", default=None, help="Waypoint name — tkinter dialog shown if omitted")
    parser.add_argument("--robodk-ip", default="localhost", help="RoboDK host (default: localhost)")
    parser.add_argument("--print-only", action="store_true",
                        help="Print joints without writing to path_config.yaml")
    args = parser.parse_args()

    rdk, robot = connect(args.robodk_ip)
    joints = get_joints(robot)

    print(f"\nRobot: {robot.Name()}")
    print(f"Joints: {format_joints(joints)}")

    if args.name is None:
        args.name = ask_name_dialog(format_joints(joints))
        if not args.name:
            print("No name given — exiting.")
            sys.exit(1)

    if args.print_only:
        print(f"\n# Add to path_config.yaml manually:")
        print(f"  {args.name}:")
        print(f"    joints: {format_joints(joints)}")
        return

    if waypoint_exists(args.name):
        print(f"\n[WARN] Waypoint '{args.name}' already exists in path_config.yaml.")
        print("  Delete or rename it first, then re-run.")
        sys.exit(1)

    append_waypoint(args.name, joints)
    print(f"\n[OK] Saved '{args.name}' to {CONFIG_PATH}")
    print(f"  Add '{args.name}' to routing_candidates: in path_config.yaml if needed.")
    _report_collision_status(args.name, REPO_ROOT)


if __name__ == "__main__":
    main()
