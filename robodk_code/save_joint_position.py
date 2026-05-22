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
    python robodk_code/save_joint_position.py --name transport_j7_0
    python robodk_code/save_joint_position.py --name curtain_safe_machine_1 --robodk-ip 172.23.208.1
    python robodk_code/save_joint_position.py --name home --print-only
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

    entry = f"\n  {name}:\n    joints: {format_joints(joints)}\n"

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


def main():
    parser = argparse.ArgumentParser(description="Save current robot joint position to path_config.yaml")
    parser.add_argument("--name", required=True, help="Waypoint name (e.g. transport_j7_0)")
    parser.add_argument("--robodk-ip", default="localhost", help="RoboDK host (default: localhost)")
    parser.add_argument("--print-only", action="store_true",
                        help="Print joints without writing to path_config.yaml")
    args = parser.parse_args()

    rdk, robot = connect(args.robodk_ip)
    joints = get_joints(robot)

    print(f"\nRobot: {robot.Name()}")
    print(f"Joints: {format_joints(joints)}")

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


if __name__ == "__main__":
    main()
