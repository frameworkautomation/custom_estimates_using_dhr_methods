"""
check_waypoint_collisions.py

For each waypoint in all_waypoints.yaml that has joints solved, sets the robot
to those joints, attaches a cone to the end effector (worst-case geometry), and
checks for static collisions. Results are written to:
    robo_dk_output/waypoint_collisions.json

Collision exclusion rules
─────────────────────────
  Target cone:   When approaching base_cone_grab_N or base_str_grab_N, the cone
                 on the tray (base_cone_N) is automatically excluded — the robot
                 is supposed to be near it. All *other* base_cone_* items are
                 checked normally (clipping adjacent cones is a real problem).

  Always-exclude: Items listed under `collision_exclude_always` in path_config.yaml
                  are disabled globally (e.g. yarn/string geometry that is always
                  OK to be near). Format: list of RoboDK item name strings.

  Attached cone: The cone_mesh_template item is repositioned to the tool before
                 each check. Collision pairs between it and the target cone are
                 disabled so the hold-and-approach geometry doesn't false-fire.

Usage:
    python robodk_code/check_waypoint_collisions.py
    python robodk_code/check_waypoint_collisions.py --robodk-ip 172.23.208.1
    python robodk_code/check_waypoint_collisions.py --force   # re-check all
    python robodk_code/check_waypoint_collisions.py --dry-run # print only, no JSON
"""

import sys
import os
import re
import json
import argparse
import datetime

sys.path.append("C:/RoboDK/Python")

REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, REPO_ROOT)

from robodk_code.waypoint_ik_utils import load_all_waypoints, resolve_output_yaml

PATH_CONFIG  = os.path.join(REPO_ROOT, "robo_dk_output", "path_config.yaml")
OUTPUT_JSON  = os.path.join(REPO_ROOT, "robo_dk_output", "waypoint_collisions.json")

# Pattern: extract slot index N from waypoint names like
#   base_cone_grab_3, base_cone_grab_3_approach, base_str_grab_3, base_str_grab_3_approach
_TARGET_CONE_RE = re.compile(r"base_(?:cone|str)_grab_(\d+)")


def target_cone_name(wp_name: str) -> str | None:
    """Return the base_cone_N name that should be excluded for this waypoint."""
    m = _TARGET_CONE_RE.search(wp_name)
    return f"base_cone_{m.group(1)}" if m else None


def load_path_config():
    try:
        import yaml
        with open(PATH_CONFIG, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        print(f"[WARN] Could not load path_config.yaml: {e}")
        return {}


def connect(ip: str):
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME
    rdk = Robolink(ip)
    robot = rdk.Item("Fanuc R2000iC 125L", ITEM_TYPE_ROBOT)
    if not robot.Valid():
        raise RuntimeError("Robot not found in station.")
    return rdk, robot


def check_collision(rdk, robot, tool, cone_mesh, joints: list,
                    target_cone_item, always_exclude_items: list) -> tuple:
    """Set robot to joints, attach cone, check collisions.

    Returns (collision: bool, colliding_names: list[str]).
    Disables: target_cone pairs, always-exclude pairs. Re-enables after.
    """
    from robodk.robolink import COLLISION_ON, COLLISION_OFF

    # Attach cone to tool so it moves with the robot
    cone_attached = cone_mesh is not None and cone_mesh.Valid()
    cone_original_parent = None
    if cone_attached:
        cone_original_parent = cone_mesh.Parent()
        cone_mesh.setParentStatic(tool)

    # Build the set of (item_a, item_b) pairs to temporarily disable
    disable_pairs = []

    if target_cone_item is not None and target_cone_item.Valid():
        # Don't count the target cone as a collision (robot is meant to approach it)
        disable_pairs.append((robot, target_cone_item))
        disable_pairs.append((tool, target_cone_item))
        if cone_attached:
            disable_pairs.append((cone_mesh, target_cone_item))

    for exc_item in always_exclude_items:
        if exc_item.Valid():
            disable_pairs.append((robot, exc_item))
            disable_pairs.append((tool, exc_item))
            if cone_attached:
                disable_pairs.append((cone_mesh, exc_item))

    for a, b in disable_pairs:
        rdk.setCollisionActivePair(COLLISION_OFF, a, b)

    robot.setJoints(joints)
    n = rdk.Collisions()

    colliding_names = []
    if n > 0:
        items = rdk.CollisionItems()
        colliding_names = [item.Name() for item in items if item.Valid()]

    for a, b in disable_pairs:
        rdk.setCollisionActivePair(COLLISION_ON, a, b)

    if cone_attached and cone_original_parent is not None:
        cone_mesh.setParentStatic(cone_original_parent)

    return n > 0, colliding_names


def main():
    parser = argparse.ArgumentParser(
        description="Static collision check for all solved waypoints in all_waypoints.yaml"
    )
    parser.add_argument("--robodk-ip", default="localhost")
    parser.add_argument("--force", action="store_true",
                        help="Re-check waypoints already in the output JSON")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print results but do not write JSON")
    args = parser.parse_args()

    yaml_path = resolve_output_yaml(REPO_ROOT)
    waypoints, _ = load_all_waypoints(yaml_path)
    print(f"Loaded {len(waypoints)} waypoints from {yaml_path}")

    # Load existing results (skip already-checked unless --force)
    existing = {}
    if os.path.exists(OUTPUT_JSON) and not args.force:
        with open(OUTPUT_JSON, "r", encoding="utf-8") as f:
            data = json.load(f)
        existing = data.get("waypoints", {})
        print(f"Existing results: {len(existing)} waypoints already checked")

    config = load_path_config()
    cone_mesh_name     = config.get("cone_mesh_template", "base_cone_0")
    tool_name          = config.get("default_tool", "pickup_closed")
    always_excl_names  = config.get("collision_exclude_always", [])

    to_check = [
        wp for wp in waypoints
        if isinstance(wp.get("joints"), list)
        and (args.force or wp["name"] not in existing)
    ]
    print(f"Waypoints to check: {len(to_check)}")

    if not to_check:
        print("Nothing to check.")
        return

    rdk, robot = connect(args.robodk_ip)
    print(f"Connected: {robot.Name()}")

    from robodk.robolink import ITEM_TYPE_TOOL, ITEM_TYPE_FRAME

    tool = rdk.Item(tool_name, ITEM_TYPE_TOOL)
    if not tool.Valid():
        print(f"[WARN] Tool '{tool_name}' not found — collision check may be incomplete")

    cone_mesh = rdk.Item(cone_mesh_name)
    if not cone_mesh.Valid():
        print(f"[WARN] cone_mesh_template '{cone_mesh_name}' not found — checking without attached cone")
        cone_mesh = None

    always_exclude_items = []
    for name in always_excl_names:
        item = rdk.Item(name)
        if item.Valid():
            always_exclude_items.append(item)
        else:
            print(f"[WARN] collision_exclude_always item not found: '{name}'")

    # Set world frame for robot
    world_frame = rdk.Item("World")
    if not world_frame.Valid():
        world_frame = rdk.Item("", ITEM_TYPE_FRAME)
    original_frame = robot.PoseFrame()
    robot.setPoseFrame(world_frame)
    robot.setTool(tool)

    results = dict(existing)
    n_clear = n_collision = n_skipped = 0

    try:
        for wp in to_check:
            name   = wp["name"]
            joints = [float(j) for j in wp["joints"]]

            tgt_name = target_cone_name(name)
            tgt_item = rdk.Item(tgt_name) if tgt_name else None
            if tgt_item is not None and not tgt_item.Valid():
                tgt_item = None  # not in station, ignore

            collision, colliding = check_collision(
                rdk, robot, tool, cone_mesh, joints,
                tgt_item, always_exclude_items
            )

            results[name] = {
                "collision":          collision,
                "colliding_items":    colliding,
                "target_cone_excluded": tgt_name,
            }

            if collision:
                n_collision += 1
                print(f"  [COLLISION]  {name}  →  {colliding}")
            else:
                n_clear += 1
                print(f"  [CLEAR]      {name}")

    finally:
        robot.setPoseFrame(original_frame)

    print(f"\n--- Summary ---")
    print(f"  Clear:      {n_clear}")
    print(f"  Collision:  {n_collision}")
    print(f"  Skipped:    {len(existing)} (already checked)")

    if not args.dry_run:
        payload = {
            "generated": datetime.datetime.now().isoformat(timespec="seconds"),
            "waypoints": results,
        }
        os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)
        with open(OUTPUT_JSON, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
        print(f"\n[OK] Written to {OUTPUT_JSON}")
    else:
        print("\n[DRY RUN] Not writing JSON.")


if __name__ == "__main__":
    main()
