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
VIEWER_CONFIG_PATH = os.path.join(REPO_ROOT, "robo_dk_output", "viewer_config.yaml")
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


def get_fk_pose(rdk, robot):
    """Return (x, y, z, rx, ry, rz) of current TCP in RoboDK world frame.

    Temporarily sets the robot reference frame to World, reads Pose(), then
    restores the previous frame.  Stored as frame: world — consistent with
    Grasshopper-exported waypoints which also use RoboDK world coordinates.
    """
    import math

    world_frame = rdk.Item("World")
    prev_frame = robot.PoseFrame()
    try:
        robot.setPoseFrame(world_frame)
        pose = robot.Pose()
    finally:
        robot.setPoseFrame(prev_frame)

    x = pose[0, 3]
    y = pose[1, 3]
    z = pose[2, 3]

    # ZYX Euler: R = Rz * Ry * Rx
    r20 = pose[2, 0]
    ry = math.asin(-max(-1.0, min(1.0, r20)))
    cos_ry = math.cos(ry)
    if abs(cos_ry) > 1e-6:
        rx = math.atan2(pose[2, 1] / cos_ry, pose[2, 2] / cos_ry)
        rz = math.atan2(pose[1, 0] / cos_ry, pose[0, 0] / cos_ry)
    else:
        # Gimbal lock
        rx = 0.0
        rz = math.atan2(-pose[0, 1], pose[1, 1])

    return (
        round(x, 3), round(y, 3), round(z, 3),
        round(math.degrees(rx), 4),
        round(math.degrees(ry), 4),
        round(math.degrees(rz), 4),
    )


def get_active_tool_name(robot):
    """Return the name of the active tool attached to the robot, or None."""
    from robodk.robolink import ITEM_TYPE_TOOL
    tool = robot.getLink(ITEM_TYPE_TOOL)
    if tool.Valid():
        return tool.Name()
    return None


def get_robot_base_world(robot):
    """Return (x, y, z) of the robot arm base in RoboDK world frame."""
    base_pose = robot.PoseAbs()
    return (
        round(base_pose[0, 3], 3),
        round(base_pose[1, 3], 3),
        round(base_pose[2, 3], 3),
    )


def update_robot_base_world(bx, by, bz):
    """Write robot_base_world: {x, y, z} into path_config.yaml.

    robot_base_world lives in path_config.yaml because it is needed for path
    planning (e.g. converting robot-local coords to world) as well as viewing.
    """
    if not os.path.exists(CONFIG_PATH):
        return
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        content = f.read()

    new_block = f"robot_base_world:\n  x: {bx}\n  y: {by}\n  z: {bz}\n"

    if "robot_base_world:" in content:
        content = re.sub(
            r"robot_base_world:\n  x:[^\n]*\n  y:[^\n]*\n  z:[^\n]*\n",
            new_block,
            content,
        )
    else:
        content = new_block + "\n" + content

    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        f.write(content)


def waypoint_exists(name):
    """Check if a waypoint with this name already exists in path_config.yaml."""
    if not os.path.exists(CONFIG_PATH):
        return False
    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        content = f.read()
    # Match "  name:" as a YAML mapping key under waypoints
    return bool(re.search(rf'^\s+{re.escape(name)}\s*:', content, re.MULTILINE))


def append_waypoint(name, joints, pose=None, tool_name=None):
    """Append a new waypoint to path_config.yaml.

    pose: optional (x, y, z, rx, ry, rz) tuple — written as Cartesian fields so
    the visualizer can show the waypoint even though it is primarily joints-driven.
    tool_name: name of the active RoboDK tool, or None.
    """
    if not os.path.exists(CONFIG_PATH):
        raise FileNotFoundError(f"path_config.yaml not found at {CONFIG_PATH}")

    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
        content = f.read()

    lines = [f"\n  {name}:"]
    if pose is not None:
        x, y, z, rx, ry, rz = pose
        lines += [
            f"    x: {x}",
            f"    y: {y}",
            f"    z: {z}",
            f"    rx: {rx}",
            f"    ry: {ry}",
            f"    rz: {rz}",
            f"    frame: world",
            f"    move_type: MoveJ",
        ]
    lines += [
        f"    tool_name: {'null' if tool_name is None else tool_name}",
        f"    joints: {format_joints(joints)}",
        f"    source: human",
    ]
    entry = "\n".join(lines) + "\n"

    # Insert before routing_candidates: so it lands in the waypoints block
    if "routing_candidates:" in content:
        content = content.replace("routing_candidates:", entry + "routing_candidates:", 1)
    elif "waypoints:" in content:
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
    parser.add_argument("--robodk-ip", default="172.23.208.1", help="RoboDK host (default: 172.23.208.1)")
    parser.add_argument("--print-only", action="store_true",
                        help="Print joints without writing to path_config.yaml")
    args = parser.parse_args()

    rdk, robot = connect(args.robodk_ip)
    joints = get_joints(robot)
    pose = get_fk_pose(rdk, robot)
    base_world = get_robot_base_world(robot)
    update_robot_base_world(*base_world)
    tool_name = get_active_tool_name(robot)

    print(f"\nRobot: {robot.Name()}")
    print(f"Tool:   {tool_name or '(none)'}")
    print(f"  Robot base (world): x={base_world[0]}  y={base_world[1]}  z={base_world[2]}")
    print(f"Joints: {format_joints(joints)}")
    print(f"Pose:   x={pose[0]}  y={pose[1]}  z={pose[2]}  rx={pose[3]}  ry={pose[4]}  rz={pose[5]}")

    if args.name is None:
        args.name = ask_name_dialog(format_joints(joints))
        if not args.name:
            print("No name given — exiting.")
            sys.exit(1)

    if args.print_only:
        print(f"\n# Add to path_config.yaml manually:")
        print(f"  {args.name}:")
        x, y, z, rx, ry, rz = pose
        print(f"    x: {x}")
        print(f"    y: {y}")
        print(f"    z: {z}")
        print(f"    rx: {rx}")
        print(f"    ry: {ry}")
        print(f"    rz: {rz}")
        print(f"    frame: world")
        print(f"    move_type: MoveJ")
        print(f"    joints: {format_joints(joints)}")
        return

    if waypoint_exists(args.name):
        print(f"\n[WARN] Waypoint '{args.name}' already exists in path_config.yaml.")
        print("  Delete or rename it first, then re-run.")
        sys.exit(1)

    append_waypoint(args.name, joints, pose=pose, tool_name=tool_name)
    print(f"\n[OK] Saved '{args.name}' to {CONFIG_PATH}")
    print(f"  Add '{args.name}' to routing_candidates: in path_config.yaml if needed.")
    _report_collision_status(args.name, REPO_ROOT)


if __name__ == "__main__":
    main()
