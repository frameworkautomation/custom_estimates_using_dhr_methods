"""
Extract specified items from a source RoboDK station into a new, clean station.

The main use case: copy the robot arm at its j7=0 position WITHOUT the linear
rail, plus any other items (tools, objects, frames) listed in the config.
This gives a repeatable starting point for movement-sequence testing.

Approach: Copy/Paste via the RoboDK API. When a 7-DOF robot (6 arm + rail) is
copied and pasted into a new station, RoboDK pastes only the 6-DOF arm (the rail
mechanism doesn't come along). We then position it at the robot's j7=0 world pose.

Usage:
    python robert_checker_stuff/extract_station.py
    python robert_checker_stuff/extract_station.py --robodk-ip 172.23.208.1
    python robert_checker_stuff/extract_station.py --source my_station.rdk --dest output.rdk
"""

import sys
import os
import json
import argparse
import math

sys.path.append("C:/RoboDK/Python")

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_TOOL, ITEM_TYPE_FRAME,
    ITEM_TYPE_TARGET, ITEM_TYPE_OBJECT,
)
from robodk.robomath import Pose_2_TxyzRxyz

# ── DEFAULTS ────────────────────────────────────────────────────────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
SAVES_DIR = os.path.join(PROJECT_DIR, "robo_dk_saves")

DEFAULT_SOURCE = os.path.join(SAVES_DIR, "for_robert_n1.rdk")
DEFAULT_DEST = os.path.join(SAVES_DIR, "for_robert_relative_to_base.rdk")
DEFAULT_CONFIG = os.path.join(SCRIPT_DIR, "station_extract_config.json")

ROBOT_NAMES = ["Fanuc R-2000iC/125L", "Fanuc R2000iC 125L"]

TYPE_MAP = {
    "robot": ITEM_TYPE_ROBOT,
    "tool": ITEM_TYPE_TOOL,
    "frame": ITEM_TYPE_FRAME,
    "target": ITEM_TYPE_TARGET,
    "object": ITEM_TYPE_OBJECT,
}


def to_robodk_path(path):
    """Convert path for RoboDK. Handles WSL /mnt/c/... → C:/... conversion."""
    abs_path = os.path.abspath(path)
    try:
        if abs_path.startswith("/mnt/"):
            parts = abs_path.split("/")
            drive = parts[2].upper()
            rest = "/".join(parts[3:])
            return f"{drive}:/{rest}"
    except (IndexError, AttributeError):
        pass
    return abs_path


# ── CONNECT ─────────────────────────────────────────────────────────────────

def connect(ip=None):
    if ip:
        return Robolink(robodk_ip=ip)
    try:
        rdk = Robolink()
        rdk.Item("")
        print("[INFO] Connected to RoboDK on localhost")
        return rdk
    except Exception:
        print("[INFO] localhost failed, trying 172.23.208.1 ...")
        return Robolink(robodk_ip="172.23.208.1")


def find_robot_in_station(RDK, name):
    """Find the robot by name, trying known aliases. Returns item or None."""
    candidates = [name] if name not in ROBOT_NAMES else ROBOT_NAMES
    for rname in candidates:
        r = RDK.Item(rname, ITEM_TYPE_ROBOT)
        if r.Valid():
            return r
    return None


# ── MAIN ────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Extract items from a source RoboDK station into a clean new station"
    )
    ap.add_argument("--source", default=DEFAULT_SOURCE,
                    help=f"Source .rdk file (default: {os.path.basename(DEFAULT_SOURCE)})")
    ap.add_argument("--dest", default=DEFAULT_DEST,
                    help=f"Destination .rdk file (default: {os.path.basename(DEFAULT_DEST)})")
    ap.add_argument("--config", default=DEFAULT_CONFIG,
                    help=f"Config JSON (default: {os.path.basename(DEFAULT_CONFIG)})")
    ap.add_argument("--robodk-ip", default=None,
                    help="RoboDK IP (default: localhost then 172.23.208.1)")
    args = ap.parse_args()

    # ── Load config ──────────────────────────────────────────────────────
    assert os.path.exists(args.config), f"Config file not found: {args.config}"
    with open(args.config, "r", encoding="utf-8") as f:
        config = json.load(f)

    items = config.get("items", [])
    assert len(items) > 0, "Config has no items to extract"

    for item_cfg in items:
        assert "name" in item_cfg, f"Item missing 'name': {item_cfg}"
        assert "type" in item_cfg, f"Item '{item_cfg['name']}' missing 'type'"
        assert item_cfg["type"] in TYPE_MAP, \
            f"Unknown type '{item_cfg['type']}' for item '{item_cfg['name']}'"

    print(f"[CONFIG] {len(items)} item(s) to extract:")
    for item_cfg in items:
        print(f"  - {item_cfg['name']} ({item_cfg['type']})")

    # ── Connect to RoboDK ────────────────────────────────────────────────
    RDK = connect(args.robodk_ip)

    # ── Load source station ──────────────────────────────────────────────
    assert os.path.exists(args.source), f"Source station not found: {args.source}"
    source_path = to_robodk_path(args.source)
    print(f"\n[LOAD] Opening source station: {source_path}")
    src_station = RDK.AddFile(source_path)
    assert src_station.Valid(), f"Failed to load source station: {args.source}"
    print(f"[LOAD] Source station loaded: '{src_station.Name()}'")

    # ── Verify all config items exist in source ──────────────────────────
    print("\n[VERIFY] Checking all items exist in source station...")
    for item_cfg in items:
        name = item_cfg["name"]
        type_str = item_cfg["type"]
        if type_str == "robot":
            robot = find_robot_in_station(RDK, name)
            assert robot is not None, \
                f"Robot '{name}' not found in source station. Tried: {ROBOT_NAMES}"
        else:
            item = RDK.Item(name, TYPE_MAP[type_str])
            assert item.Valid(), \
                f"Item '{name}' (type={type_str}) not found in source station"
        print(f"  [OK] {name} ({type_str})")
    print("[VERIFY] All items found in source")

    # ── Read source data and Copy items ──────────────────────────────────
    # We must read poses and Copy() items BEFORE creating the new station,
    # because AddStation changes the active station context.

    source_data = {}  # keyed by item name

    for item_cfg in items:
        name = item_cfg["name"]
        type_str = item_cfg["type"]

        if type_str == "robot":
            robot_src = find_robot_in_station(RDK, name)
            src_joints = robot_src.Joints()
            try:
                jlist = src_joints.list()
            except AttributeError:
                jlist = list(src_joints)

            # Set j7 to 0 to get the robot base world pose at rail home
            if len(jlist) >= 7:
                jlist_j7zero = list(jlist)
                jlist_j7zero[6] = 0.0
                robot_src.setJoints(jlist_j7zero)

            robot_base_world = robot_src.PoseAbs()
            base_txyz = Pose_2_TxyzRxyz(robot_base_world)
            print(f"\n[READ] Robot '{robot_src.Name()}' base at j7=0: "
                  f"x={base_txyz[0]:.1f} y={base_txyz[1]:.1f} z={base_txyz[2]:.1f}")

            # Copy the robot (RoboDK clipboard)
            robot_src.Copy()
            source_data[name] = {
                "type": "robot",
                "base_pose": robot_base_world,
                "base_txyz": base_txyz,
            }

        else:
            item = RDK.Item(name, TYPE_MAP[type_str])
            world_pose = item.PoseAbs()
            txyz = Pose_2_TxyzRxyz(world_pose)
            print(f"\n[READ] {type_str} '{name}': "
                  f"x={txyz[0]:.1f} y={txyz[1]:.1f} z={txyz[2]:.1f}")

            item.Copy()
            source_data[name] = {
                "type": type_str,
                "world_pose": world_pose,
            }

    # ── Create destination station ───────────────────────────────────────
    print(f"\n[CREATE] Creating new station...")
    RDK.AddStation("ExtractedStation")

    # ── Paste and position each item ─────────────────────────────────────
    for item_cfg in items:
        name = item_cfg["name"]
        type_str = item_cfg["type"]
        data = source_data[name]

        if type_str == "robot":
            base_pose = data["base_pose"]
            base_txyz = data["base_txyz"]

            print(f"\n[ROBOT] Placing robot at j7=0 world pose:")
            print(f"  x={base_txyz[0]:.1f} y={base_txyz[1]:.1f} z={base_txyz[2]:.1f} "
                  f"rx={math.degrees(base_txyz[3]):.1f} "
                  f"ry={math.degrees(base_txyz[4]):.1f} "
                  f"rz={math.degrees(base_txyz[5]):.1f}")

            # We need to re-copy from source because clipboard may be overwritten
            # by subsequent Copy() calls. For now, with only one item this is fine.
            # For multi-item: we'd need to handle this differently.
            # Paste the robot
            robot_dst = RDK.Paste()
            assert robot_dst.Valid(), "Paste() failed — no robot in clipboard"
            assert robot_dst.Type() == ITEM_TYPE_ROBOT, \
                f"Pasted item is not a robot (type={robot_dst.Type()})"

            # Check DOF — should be 6 (rail stripped by Copy/Paste)
            dst_joints = robot_dst.Joints()
            try:
                dst_jlist = dst_joints.list()
            except AttributeError:
                dst_jlist = list(dst_joints)
            print(f"[ROBOT] Pasted DOF: {len(dst_jlist)}")
            assert len(dst_jlist) == 6, \
                f"Expected 6-DOF robot but got {len(dst_jlist)} joints. " \
                f"The rail mechanism may have been included in the paste."

            # Position the robot at the j7=0 base pose.
            # Cannot use setPoseAbs on a robot — it controls TCP, not base.
            # Instead: create a parent frame at the target pose, re-parent robot under it.
            station_dst = RDK.ActiveStation()
            base_frame = RDK.AddFrame("RobotBase", station_dst)
            base_frame.setPose(base_pose)
            robot_dst.setParent(base_frame)

            # Set joints to home
            robot_dst.setJoints([0.0] * 6)

            # Verify base position
            actual_base = robot_dst.PoseAbs()
            actual_txyz = Pose_2_TxyzRxyz(actual_base)
            pos_err = math.sqrt(
                sum((actual_txyz[i] - base_txyz[i])**2 for i in range(3))
            )
            print(f"[ROBOT] Base position error: {pos_err:.2f} mm")
            assert pos_err < 1.0, \
                f"Robot base position mismatch: {pos_err:.2f} mm"

            print(f"[ROBOT] 6-DOF robot placed at j7=0 world pose")

        else:
            # Generic item — paste and reposition
            pasted = RDK.Paste()
            assert pasted.Valid(), f"Paste() failed for '{name}'"
            world_pose = data["world_pose"]
            pasted.setPoseAbs(world_pose)
            txyz = Pose_2_TxyzRxyz(world_pose)
            print(f"[{type_str.upper()}] Pasted '{pasted.Name()}' at "
                  f"x={txyz[0]:.1f} y={txyz[1]:.1f} z={txyz[2]:.1f}")

    # ── Save ─────────────────────────────────────────────────────────────
    dest_path = to_robodk_path(args.dest)
    print(f"\n[SAVE] Saving to: {dest_path}")
    RDK.Save(dest_path)

    if os.path.exists(args.dest):
        size = os.path.getsize(args.dest)
        print(f"[SAVE] Written: {args.dest} ({size:,} bytes)")
        if size < 5000:
            print(f"[WARN] File is suspiciously small ({size} bytes) — "
                  f"RoboDK free license may have truncated the save.")
    else:
        print(f"[WARN] Save file not found — save may have failed")

    # ── Final verification: query the new station ────────────────────────
    print(f"\n[VERIFY] Querying destination station...")
    robots = RDK.ItemList(ITEM_TYPE_ROBOT)
    print(f"  Robots: {[r.Name() for r in robots]}")
    assert len(robots) >= 1, "No robots in destination station!"

    frames = RDK.ItemList(ITEM_TYPE_FRAME)
    print(f"  Frames: {[f.Name() for f in frames]}")
    tools = RDK.ItemList(ITEM_TYPE_TOOL)
    print(f"  Tools:  {[t.Name() for t in tools]}")
    objects = RDK.ItemList(ITEM_TYPE_OBJECT)
    print(f"  Objects: {[o.Name() for o in objects]}")

    all_items = RDK.ItemList()
    print(f"  Total items: {len(all_items)}")

    # Verify robot has 6 DOF one more time
    for r in robots:
        try:
            nj = len(r.Joints().list())
        except AttributeError:
            nj = len(list(r.Joints()))
        print(f"  Robot '{r.Name()}': {nj} DOF")
        assert nj == 6, f"Robot '{r.Name()}' has {nj} DOF, expected 6"

    print("\n[DONE] Station extraction complete.")


if __name__ == "__main__":
    main()
