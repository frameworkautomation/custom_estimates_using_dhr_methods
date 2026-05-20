"""
load_path_plan_to_robodk.py

Reads path_plan.yaml and creates a structured set of joint targets in RoboDK
so you can visually inspect every planned position before running moving_a_cone.py.

Station tree created:
  PathPlan/
    RoutingCandidates/
      home
      transport
      ...
    BaseCones/
      base_cone_grab_0/
        approach
        grab
      ...
    DestinationGroups/
      machine_1/
        [gateway: transport]
        cone_grab_1/
          approach
          grab
        ...

Untested cones (tested: false) are skipped.
Re-running the script clears and rebuilds the PathPlan frame.

Usage:
    python robodk_code/load_path_plan_to_robodk.py
"""

import sys
import os

sys.path.append("C:/RoboDK/Python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import yaml

PLAN_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "robo_dk_output", "path_plan.yaml"
)
ROOT_FRAME_NAME = "PathPlan"


def _load_plan(path: str) -> dict:
    if not os.path.isfile(path):
        print(f"[ERROR] path_plan.yaml not found: {path}")
        print("        Run check_collision_free_paths.py first.")
        sys.exit(1)
    with open(path) as f:
        return yaml.safe_load(f)


def _clear_existing(rdk, name: str):
    """Delete existing PathPlan frame and all children if present."""
    existing = rdk.Item(name)
    if existing.Valid():
        existing.Delete()
        print(f"[INFO] Cleared existing '{name}' frame.")


def _add_frame(rdk, name: str, parent) -> object:
    """Add a child frame under parent."""
    frame = rdk.AddFrame(name, parent)
    return frame


def _add_joint_target(rdk, robot, name: str, joints: list, parent) -> object:
    """Add a joint-space target under parent and set its joint values."""
    target = rdk.AddTarget(name, parent, robot)
    target.setAsJointTarget()
    target.setJoints(joints)
    return target


def main():
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT

    # Connect
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected to RoboDK")
    except Exception as e:
        print(f"[ERROR] Cannot connect to RoboDK: {e}")
        sys.exit(1)

    # Find robot (needed to create joint targets)
    robot = None
    for item in rdk.ItemList(ITEM_TYPE_ROBOT):
        if "Mechanism" not in item.Name():
            robot = item
            break
    if robot is None:
        print("[ERROR] No robot found in station.")
        sys.exit(1)
    print(f"[INFO] Using robot: {robot.Name()}")

    plan = _load_plan(PLAN_PATH)
    print(f"[INFO] Plan generated: {plan.get('generated', '?')}")

    rdk.Render(False)
    _clear_existing(rdk, ROOT_FRAME_NAME)

    station = rdk.ActiveStation()
    root = _add_frame(rdk, ROOT_FRAME_NAME, station)

    # ── Routing candidates ────────────────────────────────────────────────────
    rc_frame = _add_frame(rdk, "RoutingCandidates", root)
    added_rc = 0
    # Infer routing candidate joint values from edge_cache (from_joints of any edge starting at that node)
    edge_cache = plan.get("edge_cache", {})
    rc_joints = {}
    for key, entry in edge_cache.items():
        from_node = key.split("|")[0]
        if from_node not in rc_joints:
            rc_joints[from_node] = entry["from_joints"]

    # Only add nodes that look like routing candidates (not approach/grab nodes)
    for name, joints in sorted(rc_joints.items()):
        if "_approach" in name or "_grab" in name:
            continue
        _add_joint_target(rdk, robot, name, joints, rc_frame)
        added_rc += 1
    print(f"[INFO] RoutingCandidates: {added_rc} targets added")

    # ── Base cones ────────────────────────────────────────────────────────────
    base_frame = _add_frame(rdk, "BaseCones", root)
    base_cones = plan.get("base_cones", {})
    added_base = 0
    skipped_base = 0
    for cone_name, cone_data in sorted(base_cones.items()):
        if not cone_data.get("tested"):
            reason = cone_data.get("reason", "unknown")
            print(f"  [SKIP] {cone_name}: {reason}")
            skipped_base += 1
            continue
        cone_frame = _add_frame(rdk, cone_name, base_frame)
        _add_joint_target(rdk, robot, "approach", cone_data["approach_joints"], cone_frame)
        _add_joint_target(rdk, robot, "grab",     cone_data["grab_joints"],     cone_frame)
        gateways = cone_data.get("gateways", [])
        print(f"  [OK]   {cone_name}  gateways={gateways}")
        added_base += 1
    print(f"[INFO] BaseCones: {added_base} added, {skipped_base} skipped")

    # ── Destination groups ────────────────────────────────────────────────────
    dest_frame = _add_frame(rdk, "DestinationGroups", root)
    dest_groups = plan.get("destination_groups", {})
    total_dest_added = 0
    total_dest_skipped = 0
    for group_name, group_data in sorted(dest_groups.items()):
        group_frame = _add_frame(rdk, group_name, dest_frame)
        gateways = group_data.get("gateways", [])

        # Add a label frame noting the gateways (can't annotate targets, so use a frame name)
        if gateways:
            _add_frame(rdk, f"[gateways: {', '.join(gateways)}]", group_frame)

        cones = group_data.get("cones", {})
        added = 0
        skipped = 0
        for cone_name, cone_data in sorted(cones.items()):
            if not cone_data.get("tested"):
                reason = cone_data.get("reason", "unknown")
                print(f"  [SKIP] {group_name}/{cone_name}: {reason}")
                skipped += 1
                continue
            cone_frame = _add_frame(rdk, cone_name, group_frame)
            _add_joint_target(rdk, robot, "approach", cone_data["approach_joints"], cone_frame)
            _add_joint_target(rdk, robot, "grab",     cone_data["grab_joints"],     cone_frame)
            added += 1
        print(f"  [INFO] {group_name}: {added} cones added, {skipped} skipped, gateways={gateways}")
        total_dest_added += added
        total_dest_skipped += skipped

    rdk.Render(True)

    print(f"\n[SUMMARY]")
    print(f"  Routing candidates : {added_rc}")
    print(f"  Base cones         : {added_base} added, {skipped_base} skipped")
    print(f"  Destination cones  : {total_dest_added} added, {total_dest_skipped} skipped")
    print(f"\n  PathPlan frame is visible in the RoboDK station tree.")


if __name__ == "__main__":
    main()
